package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

import java.util.List;
import java.util.function.Supplier;

public abstract class BaseTeleOp extends OpMode {
    public static Pose startingPose = new Pose(56.5, 8.75, Math.toRadians(90));
    protected static double drivingTolerance = 0.1;
    // Should probably add these and other poses to their own constants class later, but just here for now
    protected Pose closeShootPose = new Pose(56, 84, Math.toRadians(136)); // blue initially
    protected Gamepad driver;
    protected Gamepad operator;
    protected Supplier<Boolean> turtleMode = () -> false;
    protected Supplier<Boolean> autoAlignButton = () -> false;
    protected boolean robotCentric = false; // Robot centric off for Saharsh
    protected boolean useBrakeMode = true;
    protected boolean revFlywheel = false;
    protected Alliance alliance;
    protected boolean autonomousDriving = false;
    protected boolean prevAutonomousDriving = false;
    protected boolean currentlyAligning = false;
    protected Limelight3A limelight;
    protected Pose parkPose = new Pose(105, 33); // blue initially
    protected Pose shootPose = new Pose(11, 140); // blue initially
    protected int targetTagId = 20; //blue goal: 20, red goal: 24
    protected double headingTolerance = 1.0;
    public static double turnKF = 0.03;
    private static final PIDFController alignPIDF = new PIDFController(new PIDFCoefficients(0.0095, 0.0, 0.0, 0));

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        startingPose = (Pose) blackboard.getOrDefault("final_auton_pose", startingPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap, alliance);
        Robot.follower.setStartingPose(startingPose);
        alliance = (Alliance) blackboard.getOrDefault("alliance", Alliance.BLUE); // blue by default
        initGamepads();
    }

    /**
     * Initializes the driver and operator gamepads.
     * Driver controls the drive train and operator controls the mechanisms on the robot
     */
    public abstract void initGamepads();

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (driver == null || operator == null)
            throw new IllegalStateException("Gamepads have not been initialized");
        if (driver.a) alliance = Alliance.BLUE;
        if (driver.b) alliance = Alliance.RED;
        if (driver.xWasPressed()) robotCentric = !robotCentric;

        CommonTelemetry.drawOnlyCurrent(Robot.follower);

        CommonTelemetry.addData("Press A", "for BLUE");
        CommonTelemetry.addData("Press B", "for RED");
        CommonTelemetry.addData("Selected Alliance", alliance);
        CommonTelemetry.debug("----------------------------");
        CommonTelemetry.addData("Press X", "to toggle robot centric");
        CommonTelemetry.addData("Robot centric", robotCentric);
        CommonTelemetry.debug("----------------------------");
        CommonTelemetry.addData("start pose", Robot.follower.getPose());
        CommonTelemetry.addData("curr time", System.currentTimeMillis());
        CommonTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Mirror poses if alliance is red
        if (alliance == Alliance.RED) {
            parkPose = parkPose.mirror();
            shootPose = shootPose.mirror();
            closeShootPose = closeShootPose.mirror();
            targetTagId = 24;
        }

        Robot.follower.startTeleopDrive(useBrakeMode);
    }

    @Override
    public void loop() {
        limelight.updateRobotOrientation(Math.toDegrees(Robot.follower.getHeading()));
        prevAutonomousDriving = autonomousDriving;
        autonomousDriving = Robot.follower.isBusy();

        if (prevAutonomousDriving && !autonomousDriving) {
            CommonTelemetry.addData("breaking", "following");
            Robot.follower.startTeleopDrive(useBrakeMode);
        }

        boolean joystickMovement = (Math.abs(gamepad1.left_stick_y) >= drivingTolerance ||
                Math.abs(gamepad1.left_stick_x) >= drivingTolerance ||
                Math.abs(gamepad1.right_stick_y) >= drivingTolerance ||
                Math.abs(gamepad1.right_stick_x) >= drivingTolerance);

        // break following if something goes wrong
        if (joystickMovement && autonomousDriving) { // if driver inputs some control (through joysticks) then break
            autonomousDriving = false;
            Robot.follower.startTeleopDrive(useBrakeMode);
        }

        /*if (!autonomousDriving && !currentlyAligning && driver.dpadUpWasPressed()) { // driver dpad UP for going to shoot pose
            autonomousDriving = true;
            lineToPose(closeShootPose);
        }
         */

        if (!autonomousDriving && !currentlyAligning && driver.dpadDownWasPressed()) {// driver dpad DOWN for going to park pose
            autonomousDriving = true;
            double desiredHeading = Math.toRadians(90);
            if (Robot.follower.getHeading() > Math.PI) {
                desiredHeading = Math.toRadians(270);
            }

            lineToPose(parkPose.withHeading(desiredHeading));
        }

        // turn to shoot based on alliance and current position
        currentlyAligning = autoAlignButton.get();

        LLResult llResult = limelight.getLatestResult();
        LLResultTypes.FiducialResult targetTag = null;
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> detectedTags = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : detectedTags) {
                if (tag.getFiducialId() == targetTagId) {
                    targetTag = tag;
                }
            }

            if (targetTag != null) {
                CommonTelemetry.addData("Target x degrees", targetTag.getTargetXDegrees());
                CommonTelemetry.addData("Target y degrees", targetTag.getTargetYDegrees());
                double headingError = -targetTag.getTargetXDegrees();
                alignPIDF.updateError(headingError);
                double turn = alignPIDF.run();
                CommonTelemetry.addData("turn power", turn);

                // minimum motor power is heading still exists
                if (Math.abs(headingError) > headingTolerance) {
                    turn += (Math.signum(turn) * turnKF);
                } else {
                    // Small corrective hold, NOT zero
                    turn *= 0.2; // could cause problems, if there are problems maybe lower or remove this
                }

                // clamp output
                turn = Math.min(1.0, Math.max(-1.0, turn));

                if (currentlyAligning) {
                    Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, turn);
                }
            }
        } else {
            CommonTelemetry.addData("Limelight", "No targets found");
            if (currentlyAligning) {
                Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, 0);
            }
        }

        if (!autonomousDriving && !currentlyAligning) {
            // mecanum();
            pedroTeleop();
        }

        // Launcher controls
        if (operator.aWasPressed()) {
            Robot.queueLaunch(); // Start launchBasedOnVelocity sequence
        }

        if (!Robot.isLauncherBusy()) {
            if (operator.right_bumper) { // intake controls
                revFlywheel = false;
                Robot.spinToIntake();
            } else if (operator.left_bumper) {
                revFlywheel = false;
                Robot.spinToOuttake();
            } else {
                Robot.setIntakePower(Constants.ZERO);
                Robot.setFeederPower(Constants.ZERO);
                if (!revFlywheel) {
                    Robot.currentNonLaunchVelocity = Constants.ZERO;
                }
            }
        }

        /*
         * TECH TIP: State Machines
         * We use a "state machine" to control our launcher motor and feeder servos in this program.
         * The first step of a state machine is creating an enum that captures the different "states"
         * that our code can be in.
         * The core advantage of a state machine is that it allows us to continue to loop through all
         * of our code while only running specific code when it's necessary. We can continuously check
         * what "State" our machine is in, run the associated code, and when we are done with that step
         * move on to the next state.
         */

        // Calling State Machine for Hinge/Ramp state
        if (operator.xWasPressed()) {
            Robot.switchRampState();
        }

        // Calling State Machine for Blocker state
        if (operator.yWasPressed()) {
            Robot.switchBlockerState();
        }

        if (operator.bWasPressed()) {
            revFlywheel = !revFlywheel;
        }

        if (operator.dpadLeftWasPressed()) {
            Robot.killLauncher();
        }

        if (revFlywheel) {
            Robot.revFlywheel();
        }

        // Intake controls (can change later)
        // Right trigger rotates forward, left trigger rotates backwards
        // By subtracting, you're able to prevent them from fighting to give power to the motor
        //Robot.setIntakePower(operator.right_trigger - operator.left_trigger);

        Robot.loop();
        Robot.follower.update();
        CommonTelemetry.draw(Robot.follower);

        /*
         * Show the state and motor powers
         */
        CommonTelemetry.addData("robot x", Robot.follower.getPose().getX());
        CommonTelemetry.addData("robot y", Robot.follower.getPose().getY());
        CommonTelemetry.addData("robot heading", Robot.follower.getPose().getHeading());
        CommonTelemetry.addData("autonomous driving", autonomousDriving);
        CommonTelemetry.addData("revFlywheel", revFlywheel);
        CommonTelemetry.addData("prev auton driving", prevAutonomousDriving);
        CommonTelemetry.addData("follower is busy", Robot.follower.isBusy());

        CommonTelemetry.update();
    }

    public void mecanum() {
        double y = -driver.left_stick_y; // Remember, Y stick is reversed!
        double x = driver.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = driver.right_stick_x;

        double dampeningFactor = turtleMode.get() ? Constants.TURTLE : 1.0;
        y *= dampeningFactor;
        x *= dampeningFactor;
        rx *= dampeningFactor;

        Robot.mecanumDrive(y, x, rx);
    }

    // lines to the desired pose from the current pose with a linear heading interpolation
    public void lineToPose(Pose desiredPose) {
        Path path = new Path(new BezierLine(Robot.follower.getPose(), desiredPose));
        path.setLinearHeadingInterpolation(Robot.follower.getHeading(), desiredPose.getHeading());
        Robot.follower.followPath(path, true);
    }

    // turns to desired shooting position from the current pose
    public void turnToShoot() {
        double xDist = Robot.follower.getPose().getX() - shootPose.getX(); // blue side: 72-11 = 61, red side: 72 - 133 = -61
        double yDist = shootPose.getY() - Robot.follower.getPose().getY(); // 140-111 = 29
        double desiredHeading = Math.atan2(xDist, yDist); // blue side: (61/29) = ~0.44 rad = ~64 deg, red side: (-61/29) = ~-0.44 rad = ~-64 deg (i think the math checks out)
        if (alliance == Alliance.BLUE) desiredHeading += (Math.PI / 2);

        Robot.follower.turnTo(desiredHeading);
    }

    public void pedroTeleop() {
        double left_stick_y = gamepad1.left_stick_y * ((alliance == Alliance.BLUE && !robotCentric) ? 1 : -1);
        double left_stick_x = gamepad1.left_stick_x * ((alliance == Alliance.BLUE && !robotCentric) ? 1 : -1);
        double right_stick_x = -gamepad1.right_stick_x;

        if (turtleMode.get()) {
            left_stick_y *= (Constants.TURTLE);
            left_stick_x *= (Constants.TURTLE);
            right_stick_x *= (Constants.TURTLE);
        }

        Robot.follower.setTeleOpDrive(
                left_stick_y,
                left_stick_x,
                right_stick_x,
                robotCentric // Robot Centric,
        );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
