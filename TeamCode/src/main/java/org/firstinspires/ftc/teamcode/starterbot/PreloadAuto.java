package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
@Autonomous
public class PreloadAuto extends OpMode {
    public static int TOTAL_SHOTS = 3;
    public static long DRIVE_TIME_MS = 2000;
    private int shotsFired = 0;
    private boolean shootingComplete = false;
    private Alliance alliance = Alliance.RED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */

        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);


        // Tell the driver that initialization is complete.
        CommonTelemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.aWasPressed()) {
            alliance = Alliance.RED;
        } else if (gamepad1.bWasPressed()) {
            alliance = Alliance.BLUE;
        }

        CommonTelemetry.addData("Press O", "for BLUE");
        CommonTelemetry.addData("Press X", "for RED");
        CommonTelemetry.addData("Selected Alliance", alliance);
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        // Always advance the internal launcher state machine
        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE);

        while (!shootingComplete) {
            if (Robot.launchSequenceState == LaunchSequenceState.IDLE) {
                if (shotsFired < TOTAL_SHOTS) {
                    Robot.launchBasedOnVelocity(Constants.LAUNCHER_FAR_VELOCITY);
                    shotsFired++;
                } else {
                    shootingComplete = true;
                    Robot.launcher.setVelocity(Constants.ZERO);
                }
            }
        }

        long driveStartTime = System.currentTimeMillis();

        // Start driving after shots are done
        // Continue driving until time elapsed
        while (System.currentTimeMillis() - driveStartTime <= DRIVE_TIME_MS) {
            Robot.arcadeDrive(0.75, 0.0);
        }

        Robot.arcadeDrive(0, 0.0);

        // ----- TELEMETRY -----
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Launcher State", Robot.launchSequenceState);
        telemetry.addData("Shooting Complete", shootingComplete);
        telemetry.update();
    }
}
