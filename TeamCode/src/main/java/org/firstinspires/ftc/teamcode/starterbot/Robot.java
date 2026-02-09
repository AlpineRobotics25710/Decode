package org.firstinspires.ftc.teamcode.starterbot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.enums.BlockerState;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;
import org.firstinspires.ftc.teamcode.starterbot.enums.RampState;
import org.firstinspires.ftc.teamcode.starterbot.interpolation.Interpolator;

public class Robot {
    // Drivetrain motors
    public static DcMotor leftDrive; // Used for 2 wheel drive (omni wheels)
    public static DcMotor rightDrive; // Used for 2 wheel drive (omni wheels)

    public static DcMotor frontLeftDrive; // Used for 4 wheel mecanum drive
    public static DcMotor frontRightDrive; // Used for 4 wheel mecanum drive
    public static DcMotor backLeftDrive; // Used for 4 wheel mecanum drive
    public static DcMotor backRightDrive; // Used for 4 wheel mecanum drive

    // Launch motors
    public static DcMotorEx launcher;
    public static CRServo leftFeeder;
    public static CRServo rightFeeder;

    // Intake motors
    public static DcMotorEx leftIntake;
    public static DcMotorEx rightIntake;
    public static Servo ramp;
    public static Servo ramp2;
    public static Servo blocker;

    // States(Enums)
    public static RampState rampState;
    public static BlockerState blockerState;
    public static LaunchSequenceState launchSequenceState;

    // Pedro
    public static Follower follower;
    public static double currentNonLaunchVelocity;
    // Launcher variables
    private static double targetVelocity = 0.0; // commanded setpoint (ticks/sec)
    private static long stateStartTime;
    private static int launchesQueued = 0;
    private static boolean decreaseLauncherVel = false;
    private static Pose goalPose = Constants.GOAL_POSE.copy();

    // Prevent instantiation from other classes.
    private Robot() {
    }

    ///  Default Blue Alliance
    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, Alliance.BLUE);
    }

    public static void init(HardwareMap hardwareMap, Alliance alliance) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        //init2WheelDrive(hardwareMap); // Uncomment when drivetrain is using 2 wheel arcade drive
        initMecanumDrive(hardwareMap); // Robot is currently using mecanum drive

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "LF");
        rightFeeder = hardwareMap.get(CRServo.class, "RF");
        leftIntake = hardwareMap.get(DcMotorEx.class, "LI");
        rightIntake = hardwareMap.get(DcMotorEx.class, "RI");
        ramp = hardwareMap.get(Servo.class, "ramp");
        ramp2 = hardwareMap.get(Servo.class, "ramp2");
        blocker = hardwareMap.get(Servo.class, "blocker");

        leftIntake.setDirection(DcMotorEx.Direction.REVERSE); // Might need to switch this
        rightIntake.setDirection(DcMotorEx.Direction.FORWARD);
        ramp2.setDirection(Servo.Direction.REVERSE);// Might need to switch this

        ramp2.setDirection(Servo.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(Constants.ZERO);
        rightFeeder.setPower(Constants.ZERO);
        ramp.setPosition(Constants.RAMP_INTAKE_POS);
        ramp2.setPosition(Constants.RAMP_INTAKE_POS);
        blocker.setPosition(Constants.BLOCKER_CLOSED);

        rampState = RampState.INTAKE;
        blockerState = BlockerState.CLOSED;
        launchSequenceState = LaunchSequenceState.IDLE;

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(475, 0, 10, 12.83));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule lm : hardwareMap.getAll(LynxModule.class)) {
            lm.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (alliance == Alliance.RED) goalPose = goalPose.mirror();

        // Init follower
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        CommonTelemetry.addData("follower heading constraint", follower.getConstraints().getHeadingConstraint());
        Interpolator.init(hardwareMap.appContext);

        /*
         * Tell the driver that initialization is complete.
         */
        CommonTelemetry.addData("Status", "Initialized");
    }

    public static void initMecanumDrive(HardwareMap hardwareMap) {
        // use this method ONLY if drivetrain uses 4 powered mecanum wheels
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
    }

    public static void loop() {
        //CommonTelemetry.debug("Motors:", "Left: " + leftDrive.getPower(), "Right: " + rightDrive.getPower());
        //CommonTelemetry.debug("Servos: ", "Left: " + leftFeeder.getPower(), "Right: " + rightFeeder.getPower());

        updateLauncher();

        // launcher telemetry
        double currTPS = launcher.getVelocity(); // measured ticks/sec from encoder
        double currRADPS = launcher.getVelocity(AngleUnit.RADIANS);
        CommonTelemetry.addData("Launcher ticks/s (curr/target)", currTPS + "/" + targetVelocity);
        CommonTelemetry.addData("Launcher rad/s (curr/target)", currRADPS + "/" + tpsToRad(targetVelocity));
        CommonTelemetry.addData("Launcher rpm (curr/target)", tpsToRpm(currTPS) + "/" + radToRpm(targetVelocity));

        CommonTelemetry.addData("Ramp State", rampState.toString());
        CommonTelemetry.addData("Ramp angle", ramp.getPosition() * Constants.MAX_RAMP_DEGREES);
        CommonTelemetry.addData("Blocker State", blockerState.toString());
        CommonTelemetry.addData("Launch Sequence State", launchSequenceState.toString());
        CommonTelemetry.addData("Launches Queued", launchesQueued);
        CommonTelemetry.addData("Interpolated velocity (ticks/sec)", Interpolator.getVelocityValue(distanceToGoal()));
        CommonTelemetry.addData("Interpolated ramp angle (deg)", Interpolator.getRampValue(distanceToGoal()));
    }

    public static double tpsToRpm(double tps) {
        return (tps * 60) / Constants.LAUNCHER_MOTOR_PPR; // ppr is Constants.LAUNCHER_MOTOR_PPR
    }

    public static double radToRpm(double radps) {
        return radps / (2 * Math.PI);
    }

    public static double tpsToRad(double tps) {
        return tps * ((2 * Math.PI) / Constants.LAUNCHER_MOTOR_PPR);
    }


    public static void setFeederPower(double power) {
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
    }

    public static void setIntakePower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public static void arcadeDrive(double forward, double rotate) {
        // use this method ONLY if drivetrain uses 2 powered non-mecanum wheels
        double leftPower = forward + rotate;
        double rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    /**
     * Full mecanum drive (robot-centric).
     */
    public static void mecanumDrive(double y, double x, double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        CommonTelemetry.addData("Front Left Power: ", frontLeftPower);
        CommonTelemetry.addData("Back Left Power: ", backLeftPower);
        CommonTelemetry.addData("Front Right Power: ", frontRightPower);
        CommonTelemetry.addData("Back Right Power: ", backRightPower);

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public static void driveForward(double power) {
        mecanumDrive(power, 0, 0);
    }

    public static void spinToIntake() {
        currentNonLaunchVelocity = Constants.LAUNCHER_INTAKE_VELOCITY; // intake
        Robot.setIntakePower(Constants.INTAKE_POWER);
        Robot.setFeederPower(Constants.FEEDER_INTAKE_POWER);
    }

    public static void spinToOuttake() {
        currentNonLaunchVelocity = -Constants.LAUNCHER_INTAKE_VELOCITY; // outtake
        Robot.setIntakePower(-Constants.INTAKE_POWER);
        Robot.setFeederPower(Constants.FEEDER_POWER);
    }

    public static void stopAll() {
        Robot.setIntakePower(Constants.ZERO);
        Robot.setFeederPower(Constants.ZERO);
        Robot.killLauncher();
    }

    public static void switchRampState() {
        // State Machine for Hinge/Ramp state
        switch (rampState) {
            case INTAKE: // we are currently in INTAKE state, and want to switch states
                setRampPos(Constants.RAMP_OUTTAKE_POS);// then change to OUTTAKE state
                rampState = RampState.OUTTAKE;  // then change to OUTTAKE state
                break;

            case OUTTAKE: // we are currently in OUTTAKE state, and want to switch states
                setRampPos(Constants.RAMP_INTAKE_POS);// then change to INTAKE state
                rampState = RampState.INTAKE; // then change to INTAKE state
                break;
        }
    }

    /// Set angle of ramp in degrees. simply returns if out of bounds (nothing will happen!)
    public static void setRampAngle(double angle) {
        if (angle < 0 || angle > Constants.MAX_RAMP_DEGREES) return;
        ramp.setPosition(angle / Constants.MAX_RAMP_DEGREES);
        ramp2.setPosition(angle / Constants.MAX_RAMP_DEGREES);
    }

    /// Set ramp position. simply returns if out of bounds (nothing will happen!)
    public static void setRampPos(double pos) {
        if (pos < 0 || pos > 1) return;
        ramp.setPosition(pos);
        ramp2.setPosition(pos);
    }

    public static void switchBlockerState() {
        switch (blockerState) {
            case OPEN:
                blocker.setPosition(Constants.BLOCKER_CLOSED);
                blockerState = BlockerState.CLOSED;
                break;

            case CLOSED:
                blocker.setPosition(Constants.BLOCKER_OPEN);
                blockerState = BlockerState.OPEN;
                break;
        }
    }

    public static double distanceToGoal() {
        return follower.getPose().distanceFrom(goalPose);
    }

    public static void setDecreaseLauncherVelocity(boolean decreaseLauncherVelocity) {
        decreaseLauncherVel = decreaseLauncherVelocity;
    }

    public static void queueLaunch() {
        launchesQueued++;
    }

    public static void updateLauncher() {
        if (isLauncherBusy()) {
            updateLauncherStateMachine();
        } else if (launchesQueued > 0) {
            startLaunchSequence();
        } else {
            if (targetVelocity != currentNonLaunchVelocity) {
                targetVelocity = currentNonLaunchVelocity;
                Robot.launcher.setVelocity(currentNonLaunchVelocity);
            }

            if (launchSequenceState != LaunchSequenceState.IDLE) {
                launchSequenceState = LaunchSequenceState.IDLE;
            }
        }
    }

    public static void killLauncher() {
        Robot.launcher.setVelocity(Constants.ZERO);
        targetVelocity = Constants.ZERO;
        launchesQueued = 0;
        launchSequenceState = LaunchSequenceState.IDLE;
        currentNonLaunchVelocity = Constants.ZERO;
    }

    public static boolean isLauncherBusy() {
        return launchSequenceState != LaunchSequenceState.IDLE;
    }

    public static boolean isLaunchQueueEmpty() {
        return launchesQueued == 0;
    }

    private static void startLaunchSequence() {
        targetVelocity = Interpolator.getVelocityValue(distanceToGoal());
        Robot.launcher.setVelocity(targetVelocity);
        if (decreaseLauncherVel) targetVelocity *= 0.95;
        setRampPos(Interpolator.getRampValue(distanceToGoal()));
        launchSequenceState = LaunchSequenceState.SPINNING_UP;
        rampState = RampState.OUTTAKE;
        stateStartTime = System.currentTimeMillis();
    }

    public static void revFlywheel() {
        currentNonLaunchVelocity = Interpolator.getVelocityValue(distanceToGoal());
    }

    // TODO: NEED TO CHANGE LAUNCHER TO ALWAYS USE TICKS PER SECOND AND RAMP TO ALWAYS USE 0-1

    public static void updateLauncherStateMachine() {
        switch (launchSequenceState) {
            case SPINNING_UP:
                // Account for if the robot is moving
                setRampPos(Interpolator.getRampValue(distanceToGoal()));
                targetVelocity = Interpolator.getVelocityValue(distanceToGoal());
                if (decreaseLauncherVel) targetVelocity *= 0.95;
                Robot.launcher.setVelocity(targetVelocity);

                // shooting tolerances
                boolean reachedSpeed = Math.abs(Robot.launcher.getVelocity() - targetVelocity) <= Constants.LAUNCHER_VELOCITY_TOLERANCE;
                //boolean timedOut = System.currentTimeMillis() - stateStartTime > Constants.SPINUP_TIMEOUT_MS;
                // Remove time out to allow driver to move robot even after queueing a shot

                if (reachedSpeed) { // || timedOut) {
                    Robot.setFeederPower(Constants.FEEDER_POWER);
                    launchSequenceState = LaunchSequenceState.FEEDING;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case FEEDING:
                if (System.currentTimeMillis() - stateStartTime >= Constants.FEED_TIME_MS) {
                    Robot.setFeederPower(Constants.ZERO);
                    stateStartTime = System.currentTimeMillis();
                    launchSequenceState = LaunchSequenceState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (System.currentTimeMillis() - stateStartTime >= Constants.LAUNCH_TIME_MS) {
                    launchesQueued--;
                    // if there are more balls to shoot, then go and shoot those
                    if (launchesQueued > 0) {
                        startLaunchSequence();
                    } else {
                        launchSequenceState = LaunchSequenceState.IDLE;
                    }
                }
                break;

            case IDLE:
                break;
        }
    }
}
