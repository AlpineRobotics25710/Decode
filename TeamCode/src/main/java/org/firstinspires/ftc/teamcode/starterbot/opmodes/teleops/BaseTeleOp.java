package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops;

import static org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.PedroBaseAuto.CLOSE_SHOOTING_ANGLE_BLUE;
import static org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.PedroBaseAuto.CLOSE_SHOOTING_ANGLE_RED;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.enums.LaunchSequenceState;

import java.util.function.Supplier;

public abstract class BaseTeleOp extends OpMode {
    // Should probably add these and other poses to their own constants class later, but just here for now
    protected final Pose closeShootPoseBlue = new Pose(56, 84, CLOSE_SHOOTING_ANGLE_BLUE);
    protected final Pose closeShootPoseRed = new Pose(88, 88, CLOSE_SHOOTING_ANGLE_RED);
    protected Gamepad driver;
    protected Gamepad operator;
    protected Supplier<Boolean> turtleMode;
    protected boolean robotCentric = true;
    protected boolean useBrakeMode = true;
    protected Alliance alliance;
    protected boolean autonomousDriving = false;
    protected Pose parkPoseBlue = new Pose(107.25, 0, 0); // TODO: Need to find real values
    protected Pose parkPoseRed = new Pose(38.75, 33.25, 0); // TODO: Need to find real values

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
        alliance = Alliance.BLUE; // blue by default
        if (driver.aWasPressed()) alliance = Alliance.BLUE;
        if (driver.bWasPressed()) alliance = Alliance.RED;

        CommonTelemetry.addData("Press A", "for BLUE");
        CommonTelemetry.addData("Press B", "for RED");
        CommonTelemetry.addData("Selected Alliance", alliance);
        CommonTelemetry.addData("curr time", System.currentTimeMillis());
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

        // this line needs to be first so that you can't control the robot when it is autonomously driving
        autonomousDriving = Robot.follower.isBusy();

        // break following if something goes wrong
        if ((driver.aWasPressed() || operator.dpadUpWasPressed()) && autonomousDriving) { // driver or operator can break following
            Robot.follower.breakFollowing();
            autonomousDriving = false;
        }

        if (!autonomousDriving && driver.dpadUpWasPressed()) {
            lineToPose(alliance == Alliance.BLUE ? closeShootPoseBlue : closeShootPoseRed);
        }

        if (!autonomousDriving && driver.dpadDownWasPressed()) {
            double desiredHeading = Math.toRadians(90);
            if (Robot.follower.getHeading() > Math.PI) {
                desiredHeading = Math.toRadians(270);
            }
            parkPoseBlue = parkPoseBlue.withHeading(desiredHeading);
            parkPoseRed = parkPoseRed.withHeading(desiredHeading);
            lineToPose(alliance == Alliance.BLUE ? parkPoseBlue : parkPoseRed);
        }

        if (!autonomousDriving) {
            // mecanum();
            pedroTeleop(); // really jittery right now, probably needs to be tuned
        }

        // Launcher controls
        if (operator.bWasPressed() && Robot.launchSequenceState == LaunchSequenceState.IDLE) { // outtake controls
            Robot.setIntakePower(Constants.ZERO); // turn off intake
            Robot.launchBasedOnVelocity(Constants.LAUNCHER_FAR_VELOCITY); // Start launchBasedOnVelocity sequence
        } else if (operator.aWasPressed() && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.setIntakePower(Constants.ZERO); // turn off intake
            Robot.launchBasedOnVelocity(Constants.LAUNCHER_CLOSE_VELOCITY); // Start launchBasedOnVelocity sequence
        } else if (operator.right_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) { // intake controls
            Robot.spinToIntake();
        } else if (operator.left_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.spinToOuttake();
        } else if (!operator.right_bumper && !operator.left_bumper && Robot.launchSequenceState == LaunchSequenceState.IDLE) {
            Robot.stopAll();
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
        CommonTelemetry.addData("autonomous driving", autonomousDriving);

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
        autonomousDriving = true;
        Robot.follower.followPath(path);
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

    protected enum Alliance {BLUE, RED}
}
