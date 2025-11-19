package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import static org.firstinspires.ftc.teamcode.starterbot.Robot.switchRampState;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
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

    /**
     * Child must supply the starting pose for this auto
     */
    protected abstract Pose getStartPose();

    /**
     * Child must build all Paths / PathChains here
     */
    protected abstract void buildPaths();

    public void autonomousPathUpdate() {
        if (follower.isBusy() || currIndex >= allPaths.size()) return;

        // if shooting, loop shooting
        if (shootingActive) {
            updateShootingSequence();
            return;
        }

        // if intaking, loop intake
        if (intakeActive) {
            updateIntakeSequence();
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
            beginShootingSequence(step, shotNeeded.get(step));
            return;
        }

        // no actions are needed, just follow the path
        followPathOrPathChain(step, false);

        // Unknown type, skip it
        advancePath();
    }

    protected void startShootingBurst(int numShots, double launchVelocityTps) {
        shotsToFire = numShots;
        shotsFired = 0;
        shootingActive = true;
        switchRampState();
        Robot.launchBasedOnVelocity(launchVelocityTps);
    }

    protected boolean updateShootingBurst(double launchVelocityTps) {
        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE);

        if (!shootingActive) {
            return true;
        }

        if (Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            shotsFired++;
            if (shotsFired >= shotsToFire) {
                shootingActive = false;
                switchRampState();
                // ensure launcher stopped
                Robot.launchBasedOnVelocity(Constants.ZERO);
                return true;
            } else {
                // start next shot
                Robot.launchBasedOnVelocity(launchVelocityTps);
            }
        }
        return false;
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
        followPathOrPathChain(shootingPath, false);

        shotsToFire = 3;
        currentShotVelocity = velocity;

        waitingBeforeShooting = true;
        pathTimer.resetTimer();          // 1-second settle delay
    }

    protected void updateShootingSequence() {
        if (waitingBeforeShooting) {
            if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
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
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
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
        CommonTelemetry.addData("x", follower.getPose().getX());
        CommonTelemetry.addData("y", follower.getPose().getY());
        CommonTelemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        CommonTelemetry.addData("opmode time (s)", opmodeTimer.getElapsedTime());
        CommonTelemetry.addData("path time (s)", pathTimer.getElapsedTime());
        CommonTelemetry.addData("shooting active", shootingActive);
        CommonTelemetry.addData("shots fired", shotsFired);
        CommonTelemetry.addData("shots to fire", shotsToFire);
        CommonTelemetry.update();
    }
}
