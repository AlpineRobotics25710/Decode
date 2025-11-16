package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import static org.firstinspires.ftc.teamcode.starterbot.Robot.switchRampState;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

/**
 * Base class for all Pedro autos using the centralized Robot + Follower setup.
 * Child classes must implement:
 *  - getStartPose()
 *  - buildPaths()
 *  - autonomousPathUpdate()
 */
public abstract class PedroBaseAuto extends OpMode {

    protected Follower follower;
    protected Timer pathTimer;
    protected Timer opmodeTimer;
    protected int pathState;
    private boolean prevA = false, prevB = false;
    protected Alliance alliance = Alliance.BLUE;

    // Shooting helper state
    protected int shotsToFire = 0;
    protected int shotsFired = 0;
    protected boolean shootingActive = false;

    public static double FAR_SHOOTING_ANGLE = Math.toRadians(113.25);
    public static double CLOSE_SHOOTING_ANGLE = Math.toRadians(135);

    /** Child must supply the starting pose for this auto (already alliance-mirrored if needed). */
    protected abstract Pose getStartPose();

    /** Child must build all Paths / PathChains here, using the current alliance. */
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

    /** Helper for autos that need to know if we're on red side. */
    protected boolean isRedAlliance() {
        return alliance == Alliance.RED;
    }

    // intake helpers
    protected void startIntake() {
        Robot.spinToIntake();
    }

    protected void stopIntake() {
        Robot.stopIntake();
    }

    // Shooting helpers

    /**
     * Start a burst of N shots using Robot.launchBasedOnVelocity().
     * Call only when Robot.launchSequenceState == IDLE.
     */
    protected void startShootingBurst(int numShots, double launchVelocityTps) {
        shotsToFire = numShots;
        shotsFired = 0;
        shootingActive = true;
        switchRampState();
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
                switchRampState();
                return true;
            } else {
                // Start next shot in the burst
                Robot.launchBasedOnVelocity(launchVelocityTps);
            }
        }
        return false;
    }

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);

        // Initialize full robot, including Pedro follower
        Robot.init(hardwareMap);
        follower = Robot.follower;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // IMPORTANT: we NO LONGER build paths or set the starting pose here.
        // That will be done in start(), AFTER the alliance is selected in init_loop().
    }

    @Override
    public void init_loop() {
        // Alliance selection using gamepad
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
        opmodeTimer.resetTimer();

        // NOW build paths with the chosen alliance, and set starting pose
        buildPaths();
        follower.setStartingPose(getStartPose());

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
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Usually nothing special; Robot/SDK handles stopping
    }
}
