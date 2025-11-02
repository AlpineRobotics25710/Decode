package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@Autonomous(name = "Goal Side Auto")
public class GoalSideAuto extends OpMode {
    public static int TOTAL_SHOTS = 3;
    public static long DRIVE_TO_LAUNCH_TIME_MS = 1500;
    public static long DRIVE_OUT_OF_LAUNCH_TIME_MS = 1250;
    public static long TURN_TIME_MS = 200;
    public static long SHOT_COOLDOWN_MS = 250; // small debounce between shots

    private enum AutoState {SHOOTING, DRIVING_TO_LAUNCH, TURNING, DRIVING_OUT_OF_LAUNCH, DONE}

    private AutoState state;

    private int shotsFired;
    private boolean shotInFlight;          // true after we command a shot, until launcher returns to IDLE
    private LaunchSequenceState prevLaunchState;

    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime shotCooldown = new ElapsedTime();
    private double turning_start_time = 0;
    private double forward_start_time = 0;
    private Alliance alliance = Alliance.RED;
    private boolean prevA = false, prevB = false;

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);
        Robot.switchRampState();

        state = AutoState.DRIVING_TO_LAUNCH;
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
            case DRIVING_TO_LAUNCH: {
                if (driveTimer.milliseconds() <= DRIVE_TO_LAUNCH_TIME_MS) {
                    Robot.arcadeDrive(0.75, 0.0);
                } else {
                    Robot.arcadeDrive(0.0, 0.0);
                    state = AutoState.SHOOTING;
                }
                break;
            }

            case SHOOTING: {
                // If no shot is currently in flight and we have more to shoot, command the next one
                if (!shotInFlight && shotsFired < TOTAL_SHOTS && ls == LaunchSequenceState.IDLE
                        && shotCooldown.milliseconds() >= SHOT_COOLDOWN_MS) {
                    Robot.launchBasedOnVelocity(Constants.LAUNCHER_CLOSE_VELOCITY);
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
                    state = AutoState.TURNING;
                    driveTimer.reset();
                    turning_start_time = driveTimer.milliseconds();
                }
                break;
            }

            case TURNING: {
                if (driveTimer.milliseconds() - turning_start_time <= TURN_TIME_MS) {
                    Robot.arcadeDrive(0, ((alliance == Alliance.BLUE) ? 1 : -1) * 0.75);
                } else {
                    Robot.arcadeDrive(0.0, 0.0);
                    state = AutoState.DRIVING_OUT_OF_LAUNCH;
                    forward_start_time = driveTimer.milliseconds();
                }

                break;
            }

            case DRIVING_OUT_OF_LAUNCH: {
                if (driveTimer.milliseconds() - forward_start_time <= DRIVE_OUT_OF_LAUNCH_TIME_MS) {
                    Robot.arcadeDrive(-0.75, 0.0);
                } else {
                    Robot.arcadeDrive(0.0, 0.0);
                    state = AutoState.DONE;
                }
                break;
            }

            case DONE: {
                Robot.arcadeDrive(0.0, 0.0);
                Robot.launcher.setVelocity(Constants.ZERO);
                break;
            }
        }

        prevLaunchState = ls;

        // ----- TELEMETRY -----
        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shots", "%d / %d", shotsFired, TOTAL_SHOTS);
        telemetry.addData("Shot In Flight", shotInFlight);
        telemetry.addData("Launcher State", ls);
        telemetry.addData("Drive Timer (ms)", (int) driveTimer.milliseconds());
        telemetry.update();
    }
}
