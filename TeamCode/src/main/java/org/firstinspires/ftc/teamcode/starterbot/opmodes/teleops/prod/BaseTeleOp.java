package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.prod;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

import java.util.function.Supplier;

public abstract class BaseTeleOp extends OpMode {
    protected Gamepad driver;
    protected Gamepad operator;

    protected Supplier<Boolean> turtleMode;
    protected boolean robotCentric = true;
    protected boolean useBrakeMode = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);
        Robot.follower.setStartingPose(new Pose());
        Robot.follower.update();
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
        // See if code is updating on control hub
        CommonTelemetry.addData("Curr time", System.currentTimeMillis());
        CommonTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        Robot.follower.startTeleopDrive(useBrakeMode);
    }

    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */

        // twoWheel(); // robot currently uses 2 wheel arcade drive
        // mecanum();
        pedroTeleop(); // really jittery right now, probably needs to be tuned

        // Launcher controls
        if (operator.bWasPressed() && Robot.launchSequenceState == LaunchSequenceState.IDLE) { // outtake controls
            Robot.setIntakePower(Constants.ZERO); // turn off intake
            Robot.launchBasedOnVelocity(Constants.LAUNCHER_FAR_VELOCITY); // Start launchBasedOnVelocity sequence
        } else if (operator.aWasPressed() && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.setIntakePower(Constants.ZERO); // turn off intake
            Robot.launchBasedOnVelocity(Constants.LAUNCHER_CLOSE_VELOCITY); // Start launchBasedOnVelocity sequence
        } else if (operator.right_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) { // intake controls
            Robot.launcher.setVelocity(Constants.LAUNCHER_INTAKE_VELOCITY);
            Robot.setIntakePower(Constants.INTAKE_POWER);
            Robot.setFeederPower(-Constants.FEEDER_POWER);
        } else if (operator.left_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.launcher.setVelocity(-Constants.LAUNCHER_INTAKE_VELOCITY);
            Robot.setIntakePower(-Constants.INTAKE_POWER);
            Robot.setFeederPower(Constants.FEEDER_POWER);
        } else if (!operator.right_bumper && !operator.left_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.launcher.setVelocity(Constants.ZERO);
            Robot.setIntakePower(Constants.ZERO);
            Robot.setFeederPower(Constants.ZERO);
        }

        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE); // Keep launchBasedOnVelocity sequence going in loop

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

        // Intake controls (can change later)
        // Right trigger rotates forward, left trigger rotates backwards
        // By subtracting, you're able to prevent them from fighting to give power to the motor
        //Robot.setIntakePower(operator.right_trigger - operator.left_trigger);

        // Loop the robot
        Robot.loop();

        /*
         * Show the state and motor powers
         */
        // Set this value to something new to see if the code is updating on the control hub
        CommonTelemetry.addData("code", "updated");
        CommonTelemetry.addData("Driver Left Stick X value: ", driver.left_stick_x);
        CommonTelemetry.addData("Driver Left Stick Y value: ", driver.left_stick_y);
        CommonTelemetry.addData("Driver Right Stick X value: ", driver.right_stick_x);

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

        Robot.mecanumDrive(y, x, rx);
    }

    public void twoWheel() {
        if (turtleMode.get()) {
            Robot.arcadeDrive(driver.left_stick_y, (Constants.TURTLE) * -driver.right_stick_x);
        } else {
            Robot.arcadeDrive(driver.left_stick_y, Constants.TURN_THROTTLE * -driver.right_stick_x);
        }
    }

    public void pedroTeleop() {
        if (turtleMode.get()) {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * (Constants.TURTLE),
                    -gamepad1.left_stick_x * (Constants.TURTLE),
                    -gamepad1.right_stick_x * (Constants.TURTLE),
                    robotCentric // Robot Centric
            );
        } else {
            Robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robotCentric // Robot Centric
            );
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
