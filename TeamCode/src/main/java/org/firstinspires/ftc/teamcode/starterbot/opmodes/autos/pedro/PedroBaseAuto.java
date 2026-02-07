package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Set;


public abstract class PedroBaseAuto extends OpMode {
    protected Follower follower;
    protected Timer opmodeTimer;
    protected Timer pathTimer;

    protected LinkedList<Object> allPaths;
    protected Set<Object> shotNeeded;
    protected Set<Object> intakeNeeded;
    protected ListIterator<Object> pathIterator;
    protected Object currPath = null;

    // intake flags
    protected boolean intakeActive = false;

    // shooting flags
    protected boolean shootingActive = false;
    protected boolean waitingBeforeShooting = false;

    protected Alliance alliance = Alliance.BLUE; // By default blue

    protected boolean interrupted = false;

    /**
     * Child must supply the starting pose for this auto
     */
    protected abstract Pose getStartPose();

    protected abstract Pose getEndPose();

    /**
     * Child must build all Paths / PathChains here
     */
    protected abstract void buildPaths();

    /**
     * Any alliance-specific set up. Called in the start() method before buildPaths();
     */
    protected abstract void allianceSetup(Alliance alliance);

    public void autonomousPathUpdate() {
        //if (follower.isBusy() || currIndex >= allPaths.size()) return;
        if (interrupted) return;

        // if shooting, loop shooting
        if (waitingBeforeShooting || shootingActive) {
            updateShootingSequence();
            return;
        }

        // if intaking, loop intake
        if (intakeActive) {
            updateIntakeSequence();
            return;
        }

        // block new paths if busy or if all paths are done
        if (follower.isBusy()) {
            return;
        }

        if (!pathIterator.hasNext()) {
            return;
        }

        Object step = currPath;

        // get ready to intake
        if (intakeNeeded.contains(step)) {
            beginIntakeSequence(step);
            return;
        }

        // if you need to shoot
        if (shotNeeded.contains(step)) {
            beginShootingSequence(step);
            return;
        }

        // follow regular path
        followPathOrPathChain(step, false);
        advancePath();
    }

    protected void beginIntakeSequence(Object intakePath) {
        follower.setMaxPower(0.35);
        Robot.spinToIntake();
        followPathOrPathChain(intakePath, true);
        intakeActive = true;
    }

    // Intake actions

    protected void updateIntakeSequence() {
        if (!intakeActive) return;

        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.35) { // Add minimum 1.0s intake time after path start
            follower.setMaxPower(1.0);
            Robot.stopAll();
            intakeActive = false;
            advancePath();
        }
    }

    protected void beginShootingSequence(Object shootingPath) {
        // ((Path) shootingPath).setBrakingStrength(1.00);
        followPathOrPathChain(shootingPath, true);

        waitingBeforeShooting = true;
        shootingActive = false;
        pathTimer.resetTimer();
    }

    // Shooting actions

    protected void updateShootingSequence() {
        if (waitingBeforeShooting) {
            if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.35) {
                waitingBeforeShooting = false;

                // Angle ramp and queue three launches
                Robot.switchRampState();
                Robot.queueLaunch();
                Robot.queueLaunch();
                Robot.queueLaunch();

                shootingActive = true;
            }
            return;
        }

        if (shootingActive) {
            if (Robot.isLaunchQueueEmpty() && !Robot.isLauncherBusy()) {
                shootingActive = false;
                Robot.switchRampState();
                advancePath();
            }
        }
    }

    protected void advancePath() {
        currPath = pathIterator.next();
        pathTimer.resetTimer();
    }

    protected void followPathOrPathChain(Object toFollow, boolean holdEnd) {
        if (toFollow instanceof Path) {
            follower.followPath((Path) toFollow, holdEnd);
        } else if (toFollow instanceof PathChain) {
            follower.followPath((PathChain) toFollow, holdEnd);
        }
    }

    public void interruptAndPark() {
        Pose followerPose = follower.getPose();

        cancelAllActions();

        Path goToEnd = new Path(new BezierLine(followerPose, getEndPose()));
        goToEnd.setLinearHeadingInterpolation(followerPose.getHeading(), getEndPose().getHeading());
        followPathOrPathChain(goToEnd, true);
    }

    protected void cancelAllActions() {
        intakeActive = false;
        waitingBeforeShooting = false;
        shootingActive = false;

        follower.breakFollowing();
        Robot.stopAll();
    }

    protected void addPath(Object path) {
        allPaths.add(path);
    }

    protected void addShot(Object path) {
        shotNeeded.add(path);
    }

    protected void addIntake(Object path) {
        intakeNeeded.add(path);
    }

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);

        allPaths = new LinkedList<>();
        shotNeeded = new HashSet<>();
        intakeNeeded = new HashSet<>();

        // Initialize full robot, including Pedro follower
        Robot.init(hardwareMap);
        follower = Robot.follower;

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pathTimer = new Timer();
        pathTimer.resetTimer();

        CommonTelemetry.addData("Autonomous", "initialized");
        CommonTelemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;

        CommonTelemetry.addData("Instructions", "Select A for BLUE, Select B for RED");
        CommonTelemetry.addData("Selected Alliance", alliance);
        CommonTelemetry.update();
    }

    @Override
    public void start() {
        allianceSetup(alliance); // Any alliance-specific set up
        follower.setStartingPose(getStartPose());
        buildPaths();
        follower.update();
        if (allPaths.isEmpty()) {
            CommonTelemetry.debug("No paths were added");
            CommonTelemetry.update();
            stop();
        }
        pathIterator = allPaths.listIterator();
        currPath = pathIterator.next();
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        // Common follower update
        follower.update();

        if (opmodeTimer.getElapsedTimeSeconds() >= 28.75 && !interrupted) {
            interrupted = true;
            follower.breakFollowing();
            interruptAndPark();
        }

        if (!interrupted) {
            autonomousPathUpdate();

            Robot.loop();
        }

        CommonTelemetry.draw(follower);

        // Common Pedro telemetry
        CommonTelemetry.addData("intake active", intakeActive);
        CommonTelemetry.addData("Waiting before shooting", waitingBeforeShooting);
        CommonTelemetry.addData("shooting active", shootingActive);
        CommonTelemetry.addData("follower busy", follower.isBusy());
        CommonTelemetry.addData("interrupted", interrupted);
        CommonTelemetry.addData("opmode time (s)", opmodeTimer.getElapsedTime());
        CommonTelemetry.addData("path time (s)", pathTimer.getElapsedTime());
        CommonTelemetry.addData("x", follower.getPose().getX());
        CommonTelemetry.addData("y", follower.getPose().getY());
        CommonTelemetry.addData("curr heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        CommonTelemetry.addData("target heading (deg)", Math.toDegrees(follower.getCurrentPath().getHeadingGoal(1.0)));
//        CommonTelemetry.addData("heading error (deg)", Math.toDegrees(follower.getHeadingError()));
        CommonTelemetry.addData("heading constraint (deg)", Math.toDegrees(follower.getConstraints().getHeadingConstraint()));
        CommonTelemetry.update();
    }

    @Override
    public void stop() {
        blackboard.put("final_auton_pose", follower.getPose());
        blackboard.put("alliance", alliance);
    }
}
