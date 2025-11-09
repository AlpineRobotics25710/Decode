package org.firstinspires.ftc.teamcode.starterbot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.starterbot.enums.BlockerState;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;
import org.firstinspires.ftc.teamcode.starterbot.enums.RampState;

import java.util.Locale;

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
    public static Servo blocker;

    // States(Enums)
    static RampState rampState;
    static BlockerState blockerState;
    public static LaunchSequenceState launchSequenceState;
    private static double targetVelocityTps = 0.0; // commanded setpoint (ticks/sec)

    private static long stateStartTime;

    // Prevent instantiation from other classes.
    private Robot() {
    }

    public static void init(HardwareMap hardwareMap) {
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
        blocker = hardwareMap.get(Servo.class, "blocker");

        leftIntake.setDirection(DcMotorEx.Direction.REVERSE); // Might need to switch this
        rightIntake.setDirection(DcMotorEx.Direction.FORWARD); // Might need to switch this

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
        blocker.setPosition(Constants.BLOCKER_CLOSED);

        rampState = RampState.INTAKE;
        blockerState = BlockerState.CLOSED;
        launchSequenceState = LaunchSequenceState.IDLE;

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        CommonTelemetry.addData("Status", "Initialized");
    }

    private static void init2WheelDrive(HardwareMap hardwareMap) {
        // use this method ONLY if drivetrain uses 2 powered non-mecanum wheels
        leftDrive = hardwareMap.get(DcMotor.class, "LD");
        rightDrive = hardwareMap.get(DcMotor.class, "RD");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
    }

    public static void initMecanumDrive(HardwareMap hardwareMap) {
        // use this method ONLY if drivetrain uses 4 powered mecanum wheels
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
    }

    public static void loop() {
        //CommonTelemetry.debug("Motors:", "Left: " + leftDrive.getPower(), "Right: " + rightDrive.getPower());
        //CommonTelemetry.debug("Servos: ", "Left: " + leftFeeder.getPower(), "Right: " + rightFeeder.getPower());

        // launcher telemetry
        double curTps = launcher.getVelocity(); // measured ticks/sec from encoder
        CommonTelemetry.addData("Launcher tps (cur/target)", String.format(Locale.US, "%.0f / %.0f", curTps, targetVelocityTps));

        // estimated RPM (5203 @ ~537.7 ticks/rev)
        CommonTelemetry.addData("Launcher rpm (est)", String.format(Locale.US, "%.0f", curTps * 60.0 / 537.7));

        CommonTelemetry.addData("Ramp State", rampState.toString());
        CommonTelemetry.addData("Blocker State", blockerState.toString());
        CommonTelemetry.addData("Launch Sequence State", launchSequenceState.toString());
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

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    // Helpers for Auto
    public static void stopDrive() {
        mecanumDrive(0, 0, 0);
    }

    public static void driveForward(double p) {
        mecanumDrive(p, 0, 0);
    }

    public static void strafeRight(double p) {
        mecanumDrive(0, p, 0);
    }

    public static void rotateRight(double p) {
        mecanumDrive(0, 0, p);
    }

    public static void strafeLeft(double p) {
        mecanumDrive(0, -p, 0);
    }

    public static void rotateLeft(double p) {
        mecanumDrive(0, 0, -p);
    }

    public static void switchRampState() {
        // State Machine for Hinge/Ramp state
        switch (rampState) {
            case INTAKE: // we are currently in INTAKE state, and want to switch states
                ramp.setPosition(Constants.RAMP_OUTTAKE_POS); // then change to OUTTAKE state
                rampState = RampState.OUTTAKE;  // then change to OUTTAKE state
                break;

            case OUTTAKE: // we are currently in OUTTAKE state, and want to switch states
                ramp.setPosition(Constants.RAMP_INTAKE_POS); // then change to INTAKE state
                rampState = RampState.INTAKE; // then change to INTAKE state
                break;
        }
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

    public static void launchTimeDelay(double launcherVelocity) {
        if (launcherVelocity != Constants.CONTINUE_LAUNCH_SEQUENCE && launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.launcher.setVelocity(launcherVelocity);
            launchSequenceState = LaunchSequenceState.SPINNING_UP;
            stateStartTime = System.currentTimeMillis();
        }

        switch (launchSequenceState) {
            case SPINNING_UP:
                if (System.currentTimeMillis() - stateStartTime >= Constants.LAUNCH_DELAY_MS) {
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
                    Robot.launcher.setVelocity(Constants.ZERO);
                    launchSequenceState = LaunchSequenceState.IDLE;
                }
                break;
        }
    }

    public static void launchBasedOnVelocity(double launcherVelocity) {
        if (launcherVelocity != Constants.CONTINUE_LAUNCH_SEQUENCE && launchSequenceState == LaunchSequenceState.IDLE) {
            targetVelocityTps = launcherVelocity;
            Robot.launcher.setVelocity(launcherVelocity);
            launchSequenceState = LaunchSequenceState.SPINNING_UP;
            stateStartTime = System.currentTimeMillis();
        }

        switch (launchSequenceState) {
            case SPINNING_UP:
                if (Robot.launcher.getVelocity() >= targetVelocityTps - Constants.LAUNCHER_VELOCITY_TOLERANCE
                        && System.currentTimeMillis() - stateStartTime < Constants.SPINUP_TIMEOUT_MS) {
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
                    Robot.launcher.setVelocity(Constants.ZERO);
                    launchSequenceState = LaunchSequenceState.IDLE;
                }
                break;
        }
    }
}
