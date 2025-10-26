package org.firstinspires.ftc.teamcode.starterbot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    // Drivetrain motors
    public static DcMotor leftDrive;
    public static DcMotor rightDrive;

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
    static LaunchSequenceState launchSequenceState;

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
        //leftDrive = hardwareMap.get(DcMotor.class, "LD");
        //rightDrive = hardwareMap.get(DcMotor.class, "RD");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "LF");
        rightFeeder = hardwareMap.get(CRServo.class, "RF");
        leftIntake = hardwareMap.get(DcMotorEx.class, "LI");
        rightIntake = hardwareMap.get(DcMotorEx.class, "RI");
        ramp = hardwareMap.get(Servo.class, "ramp");
        blocker = hardwareMap.get(Servo.class, "blocker");


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

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
        //leftDrive.setZeroPowerBehavior(BRAKE);
        //rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(Constants.STOP_POWER);
        rightFeeder.setPower(Constants.STOP_POWER);
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
        CommonTelemetry.update();
    }

    public static void loop() {
        //CommonTelemetry.debug("Motors:", "Left: " + leftDrive.getPower(), "Right: " + rightDrive.getPower());
        CommonTelemetry.debug("Servos: ", "Left: " + leftFeeder.getPower(), "Right: " + rightFeeder.getPower());
        CommonTelemetry.addData("Launcher speed", launcher.getVelocity());
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
        double leftPower = forward + rotate;
        double rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        //leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);
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

    public static void launch(double launcherVelocity) {
        if (launcherVelocity != Constants.CONTINUE_LAUNCH_SEQUENCE && launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.launcher.setVelocity(launcherVelocity);
            launchSequenceState = LaunchSequenceState.SPINNING_UP;
            stateStartTime = System.currentTimeMillis();
        }

        switch (launchSequenceState) {
            case SPINNING_UP:
                if (System.currentTimeMillis() - stateStartTime >= Constants.LAUNCH_DELAY_MS) {
                    Robot.setFeederPower(Constants.FULL_POWER);
                    launchSequenceState = LaunchSequenceState.FEEDING;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case FEEDING:
                if (System.currentTimeMillis() - stateStartTime >= Constants.FEED_TIME_MS) {
                    Robot.setFeederPower(0.0);
                    stateStartTime = System.currentTimeMillis();
                    launchSequenceState = LaunchSequenceState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (System.currentTimeMillis() - stateStartTime >= Constants.LAUNCH_TIME_MS) {
                    Robot.launcher.setVelocity(Constants.STOP_POWER);
                    launchSequenceState = LaunchSequenceState.IDLE;
                }
                break;
        }
    }
}
