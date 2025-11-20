package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.timebased;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

@Configurable
@Autonomous(name = "Preload Auto", group = "Time-Based Auto")
public class PreloadAuto extends OpMode {
    public static int TOTAL_SHOTS = 3;
    public static long DRIVE_TIME_MS = 250;
    public static long SHOT_COOLDOWN_MS = 250; // small debounce between shots
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime shotCooldown = new ElapsedTime();
    private AutoState state;
    private int shotsFired;
    private boolean shotInFlight;          // true after we command a shot, until launcher returns to IDLE
    private LaunchSequenceState prevLaunchState;
    private Alliance alliance = Alliance.RED;
    private boolean prevA = false, prevB = false;

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);
        Robot.switchRampState();

        state = AutoState.SHOOTING;
        shotsFired = 0;
        shotInFlight = false;
        prevLaunchState = LaunchSequenceState.IDLE;
        shotCooldown.reset();

        CommonTelemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Edge-detect if you don't have aWasPressed/bWasPressed helpers
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (a && !prevA) alliance = Alliance.RED;
        if (b && !prevB) alliance = Alliance.BLUE;
        prevA = a;
        prevB = b;

        CommonTelemetry.addData("Press B/O", "for BLUE");
        CommonTelemetry.addData("Press A/X", "for RED");
        CommonTelemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {
        driveTimer.reset();
        shotCooldown.reset();
    }

    @Override
    public void loop() {
        // Always tick the launcher state machine
        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE);

        LaunchSequenceState ls = Robot.launchSequenceState;

        switch (state) {
            case SHOOTING: {
                // If no shot is currently in flight and we have more to shoot, command the next one
                if (!shotInFlight && shotsFired < TOTAL_SHOTS && ls == LaunchSequenceState.IDLE
                        && shotCooldown.milliseconds() >= SHOT_COOLDOWN_MS) {
                    Robot.launchBasedOnVelocity(Constants.LAUNCHER_FAR_VELOCITY);
                    shotInFlight = true;          // wait for cycle to complete
                    shotCooldown.reset();
                }

                // Detect end of a shot cycle by seeing the launcher return to IDLE
                // (falling back to IDLE after SHOOTING in your Robot state machine)
                if (shotInFlight && ls == LaunchSequenceState.IDLE && prevLaunchState != LaunchSequenceState.IDLE) {
                    shotsFired++;
                    shotInFlight = false;
                }

                // When all shots are completed and launcher is idle, transition to driving
                if (shotsFired >= TOTAL_SHOTS && ls == LaunchSequenceState.IDLE) {
                    Robot.launcher.setVelocity(Constants.ZERO);
                    state = AutoState.DRIVING;
                    driveTimer.reset();
                }
                break;
            }

            case DRIVING: {
                if (driveTimer.milliseconds() <= DRIVE_TIME_MS) {
                    Robot.driveForward(0.6);
                } else {
                    Robot.driveForward(0);
                    state = AutoState.DONE;
                }
                break;
            }

            case DONE: {
                Robot.driveForward(0.0);
                Robot.launcher.setVelocity(Constants.ZERO);
                break;
            }
        }

        prevLaunchState = ls;

        // telemetry
        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shots", "%d / %d", shotsFired, TOTAL_SHOTS);
        telemetry.addData("Shot In Flight", shotInFlight);
        telemetry.addData("Launcher State", ls);
        telemetry.addData("Drive Timer (ms)", (int) driveTimer.milliseconds());
        telemetry.update();
    }

    private enum AutoState {SHOOTING, DRIVING, DONE}
}
