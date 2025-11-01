package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
@Autonomous
public class ShootAndLeaveAuto extends LinearOpMode {
    private static final int TOTAL_SHOTS = 3;
    private static final long WAIT_BETWEEN_SHOTS_MS = 800;
    private static final long DRIVE_TIME_MS = 700;

    private int shotsFired = 0;
    private long lastShotTime = 0;
    private long driveStartTime = 0;

    private boolean shootingComplete = false;
    private boolean driving = false;

    @Override
    public void runOpMode() {

        waitForStart();

        long currentTime = System.currentTimeMillis();

        // Always advance the internal launcher state machine
        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE);

        if (!shootingComplete) {
            if (Robot.launchSequenceState == LaunchSequenceState.IDLE) {
                if (shotsFired < TOTAL_SHOTS && currentTime - lastShotTime >= WAIT_BETWEEN_SHOTS_MS) {
                    Robot.launchBasedOnVelocity(Constants.LAUNCHER_FAR_VELOCITY);
                    shotsFired++;
                    lastShotTime = currentTime;
                } else if (shotsFired >= TOTAL_SHOTS) {
                    shootingComplete = true;
                    Robot.launcher.setVelocity(Constants.ZERO);
                }
            }
        } else if (!driving) {
            // Start driving after shots are done
            driveStartTime = currentTime;
            driving = true;
            Robot.arcadeDrive(0.75, 0.0);
        } else if (driving) {
            // Continue driving until time elapsed
            if (currentTime - driveStartTime >= DRIVE_TIME_MS) {
                Robot.arcadeDrive(0.0, 0.0);
                driving = false;
            }
        }

        // ----- TELEMETRY -----
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Launcher State", Robot.launchSequenceState);
        telemetry.addData("Shooting Complete", shootingComplete);
        telemetry.addData("Driving", driving);
        telemetry.update();
    }
}
