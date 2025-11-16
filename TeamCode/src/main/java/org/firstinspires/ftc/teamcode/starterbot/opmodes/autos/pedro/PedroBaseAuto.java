package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

/**
 * Base class for all Pedro autos using the centralized Robot + Follower setup.
 * Child classes must implement:
 *  - getStartPose()
 *  - buildPaths()
 *  - autonomousPathUpdate()
 */
public abstract class PedroAutoBase extends OpMode {

    protected Follower follower;
    protected Timer pathTimer;
    protected Timer opmodeTimer;
    protected int pathState;

    // Shooting helper state
    protected int shotsToFire = 0;
    protected int shotsFired = 0;
    protected boolean shootingActive = false;

    // Tune these if needed
    protected static final double AUTO_LAUNCH_VELOCITY_TPS = 3000.0; // guess, tune on bot
    protected static final double AUTO_INTAKE_POWER = 1.0;

    /** Child must supply the starting pose for this auto. */
    protected abstract Pose getStartPose();

    /** Child must build all Paths / PathChains here. */
    protected abstract void buildPaths();

    /** Child must implement the path state machine here. */
    protected abstract void autonomousPathUpdate();

    /** Common helper to change path state + reset path timer. */
    protected void setPathState(int newState) {
        pathState = newState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    // ==== Intake helpers ====
    protected void startIntake() {
        Robot.setIntakePower(AUTO_INTAKE_POWER);
    }

    protected void stopIntake() {
        Robot.setIntakePower(0.0); // or Constants.ZERO if you prefer
    }

    // ==== Shooting helpers ====

    /**
     * Start a burst of N shots using Robot.launchBasedOnVelocity().
     * Call only when Robot.launchSequenceState == IDLE.
     */
    protected void startShootingBurst(int numShots, double launchVelocityTps) {
        shotsToFire = numShots;
        shotsFired = 0;
        shootingActive = true;
        Robot.launchBasedOnVelocity(launchVelocityTps);
    }

    /**
     * Call this every loop while in a "shooting" state.
     * Returns true when the burst is completely done.
     */
    protected boolean updateShootingBurst(double launchVelocityTps) {
        // Keep the current launch sequence progressing
        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE);

        if (!shootingActive) {
            return true;
        }

        if (Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            shotsFired++;
            if (shotsFired >= shotsToFire) {
                shootingActive = false;
                return true;
            } else {
                // Start next shot in the burst
                Robot.launchBasedOnVelocity(launchVelocityTps);
            }
        }
        return false;
    }

    // ==== OpMode lifecycle ====

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);

        // Initialize full robot, including Pedro follower
        Robot.init(hardwareMap);
        follower = Robot.follower;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths();
        follower.setStartingPose(getStartPose());
    }

    @Override
    public void init_loop() {
        // optional: child can override if needed
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Common follower update
        follower.update();

        // Let child drive the state machine
        autonomousPathUpdate();

        // Common Pedro telemetry
        telemetry.addData("pathState", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Usually nothing special; Robot/SDK handles stopping
    }
}
