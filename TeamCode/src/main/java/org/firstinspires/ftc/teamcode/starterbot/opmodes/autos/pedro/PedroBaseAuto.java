package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;


public abstract class PedroBaseAuto extends OpMode {
    protected Follower follower;
    protected Timer opmodeTimer;
    protected Timer pathTimer;

    protected LinkedList<Object> allPaths;
    protected Map<Object, Double> shotNeeded;
    protected Set<Object> intakeNeeded;
    protected int currIndex = 0;

    // intake flags
    protected boolean intakeActive = false;

    // shooting flags
    protected int shotsToFire = 0;
    protected int shotsFired = 0;
    protected boolean shootingActive = false;
    protected boolean waitingBeforeShooting = false;
    protected double currentShotVelocity = 0;

    protected Alliance alliance = Alliance.BLUE; // By default blue

    /**
     * Child must supply the starting pose for this auto
     */
    protected abstract Pose getStartPose();

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

        if (currIndex >= allPaths.size()) {
            return;
        }

        Object step = currentPath();

        // get ready to intake
        if (intakeNeeded.contains(step)) {
            beginIntakeSequence(step);
            return;
        }

        // if you need to shoot
        if (shotNeeded.containsKey(step)) {
            Double velocity = shotNeeded.get(step);
            if (velocity != null) { // ensure not null
                beginShootingSequence(step, velocity);
            }
            return;
        }

        // follow regular path
        followPathOrPathChain(step, false);
        advancePath();
    }

    // Intake actions

    protected void beginIntakeSequence(Object intakePath) {
        follower.setMaxPower(0.5);
        Robot.spinToIntake();
        followPathOrPathChain(intakePath, true);
        intakeActive = true;
    }

    protected void updateIntakeSequence() {
        if (!intakeActive) return;

        if (!follower.isBusy()) {
            follower.setMaxPower(1.0);
            Robot.stopAll();
            intakeActive = false;
            advancePath();
        }
    }

    // Shooting actions

    protected void beginShootingSequence(Object shootingPath, double velocity) {
        followPathOrPathChain(shootingPath, true);

        shotsToFire = 3;
        shotsFired = 0;
        currentShotVelocity = velocity;
        waitingBeforeShooting = true;
        shootingActive = false;
        pathTimer.resetTimer();          // 1-second settle delay
    }

    protected void updateShootingSequence() {
        if (waitingBeforeShooting) {
            if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                waitingBeforeShooting = false;

                Robot.switchRampState();
                shootingActive = true;
                shotsFired = 0;
            }
            return;
        }

        if (shootingActive) {
            Robot.launchBasedOnVelocity(currentShotVelocity);

            if (Robot.launchSequenceState == LaunchSequenceState.IDLE) {
                shotsFired++;

                if (shotsFired >= shotsToFire) {
                    shootingActive = false;
                    Robot.switchRampState();
                    advancePath();
                } else {
                    Robot.launchBasedOnVelocity(currentShotVelocity);
                }
            }
        }
    }

    protected Object currentPath() {
        return safeGet(currIndex);
    }

    protected void advancePath() {
        currIndex++;
        pathTimer.resetTimer();
    }

    protected Object safeGet(int index) {
        if (index < 0 || index >= allPaths.size()) return null;
        return allPaths.get(index);
    }

    protected void followPathOrPathChain(Object toFollow, boolean holdEnd) {
        if (toFollow instanceof Path) {
            follower.followPath((Path) toFollow, holdEnd);
        } else if (toFollow instanceof PathChain) {
            follower.followPath((PathChain) toFollow, holdEnd);
        }
    }

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);

        allPaths = new LinkedList<>();
        shotNeeded = new HashMap<>();
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
        opmodeTimer.resetTimer();
        allianceSetup(alliance);
        buildPaths();
        follower.setStartingPose(getStartPose());
    }

    @Override
    public void loop() {
        // Common follower update
        follower.update();

        // Let child drive the state machine
        autonomousPathUpdate();

        Robot.loop();

        // Common Pedro telemetry
        CommonTelemetry.addData("curr index", currIndex);
        CommonTelemetry.addData("intake active", intakeActive);
        CommonTelemetry.addData("Waiting before shooting", waitingBeforeShooting);
        CommonTelemetry.addData("shooting active", shootingActive);
        CommonTelemetry.addData("shots fired", shotsFired);
        CommonTelemetry.addData("shots to fire", shotsToFire);
        CommonTelemetry.addData("follower busy", follower.isBusy());
        CommonTelemetry.addData("opmode time (s)", opmodeTimer.getElapsedTime());
        CommonTelemetry.addData("path time (s)", pathTimer.getElapsedTime());
        CommonTelemetry.addData("x", follower.getPose().getX());
        CommonTelemetry.addData("y", follower.getPose().getY());
        CommonTelemetry.addData("curr heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        CommonTelemetry.addData("target heading (deg)", Math.toDegrees(follower.getCurrentPath().getHeadingGoal(1.0)));
        CommonTelemetry.addData("heading error (rad)", follower.getHeadingError());
        CommonTelemetry.addData("heading constraint (rad)", follower.getConstraints().getHeadingConstraint());
        CommonTelemetry.update();
    }

    @Override
    public void stop() {
        blackboard.put("final_auton_pose", follower.getPose());
    }
}
