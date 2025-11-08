package org.firstinspires.ftc.teamcode.starterbot.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

public abstract class BaseTeleOp extends OpMode {
    protected Gamepad driver;
    protected Gamepad operator;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);
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

        twoWheel(driver); // robot currently uses 2 wheel arcade drive
        // mecanum(driver); // TODO: Uncomment when drivetrain is switched to mecanum wheels

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
        CommonTelemetry.update();
    }

    public void mecanum(Gamepad driver) {
        double f = driver.left_stick_y; // forward/back
        double s = driver.left_stick_x; // strafe
        double r = -driver.right_stick_x * (driver.right_bumper ? Constants.TURTLE : Constants.TURN_THROTTLE);

        Robot.mecanumDrive(f, s, r);
    }

    public void twoWheel(Gamepad driver) {
        if (driver.right_bumper) {
            Robot.arcadeDrive(driver.left_stick_y, (Constants.TURTLE) * -driver.right_stick_x);
        } else {
            Robot.arcadeDrive(driver.left_stick_y, Constants.TURN_THROTTLE * -driver.right_stick_x);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
