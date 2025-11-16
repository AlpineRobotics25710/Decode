package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

/**
 * Base class for all Pedro autos using the centralized Robot + Follower setup.
 * Child classes only need to implement:
 *  - getStartPose()
 *  - buildPaths()
 *  - autonomousPathUpdate()
 */
public abstract class PedroBaseAuto extends OpMode {

    protected Follower follower;
    protected Timer pathTimer;
    protected Timer opmodeTimer;
    protected int pathState;

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
