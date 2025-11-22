package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.prod;

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
    protected Gamepad driver;
    protected Gamepad operator;

    protected Supplier<Boolean> turtleMode;
    protected boolean robotCentric = true;
    protected boolean useBrakeMode = true;

    protected Alliance alliance;
    protected boolean autonomousDriving = false;

    protected enum Alliance {BLUE, RED}

    // Poses (x, y, heading) – heading is ALWAYS in radians
    protected Pose closeShootPose = new Pose(88, 88, CLOSE_SHOOTING_ANGLE_RED); // Red Side Close Shooting Pose by default
    protected double parkHeading = Math.toRadians(90); // radians
    protected Pose parkPose = new Pose(38.75, 33.25, parkHeading); // Red Side Park Pose by default

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

        alliance = Alliance.RED; // red by default
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
        if (driver.aWasPressed() && alliance != Alliance.BLUE) {
            parkPose.mirror();
            closeShootPose.mirror();
            closeShootPose = closeShootPose.withHeading(CLOSE_SHOOTING_ANGLE_BLUE);
            alliance = Alliance.BLUE;
        }

        if (driver.bWasPressed() && alliance != Alliance.RED) {
            parkPose.mirror();
            closeShootPose.mirror();
            closeShootPose = closeShootPose.withHeading(CLOSE_SHOOTING_ANGLE_RED);
            alliance = Alliance.RED;
        }

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
        // 1) Sync our flag with Pedro's actual state
        autonomousDriving = Robot.follower.isBusy();

        // 2) Allow either driver or operator to cancel following if something goes wrong
        if ((driver.aWasPressed() || operator.dpadUpWasPressed()) && autonomousDriving) { // driver or operator can break following
            Robot.follower.breakFollowing();
            autonomousDriving = false;
        }

        // 3) Handle waypoint requests ONLY when not currently auto-driving

        // Line to close shooting pose from current pose
        if (!autonomousDriving && driver.dpadUpWasPressed()) {
            lineToPose(closeShootPose);
            autonomousDriving = true; // block TeleOp drive this loop as we just started a path
        }

        // Line to park pose from current pose
        if (!autonomousDriving && driver.dpadDownWasPressed()) {
            // Decide whether to park facing +Y (90) or -Y (270) based on current heading
            double currentHeading = Robot.follower.getHeading(); // radians

            if (currentHeading > Math.PI) {
                parkHeading = Math.toRadians(270);
            } else {
                parkHeading = Math.toRadians(90);
            }

            parkPose = parkPose.withHeading(parkHeading);
            lineToPose(parkPose);
            autonomousDriving = true; // block TeleOp drive this loop as we just started a path
        }

        // 4) Manual TeleOp drive ONLY when not following a path
        if (!autonomousDriving) {
            // mecanum();
            pedroTeleop();
        }

        // 5) Launcher controls
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
            Robot.stopIntake();
        }

        Robot.launchBasedOnVelocity(Constants.CONTINUE_LAUNCH_SEQUENCE); // Keep launchBasedOnVelocity sequence going in loop

        // Calling State Machine for Hinge/Ramp state
        if (operator.xWasPressed()) {
            Robot.switchRampState();
        }

        // Calling State Machine for Blocker state
        if (operator.yWasPressed()) {
            Robot.switchBlockerState();
        }

        // Loop the robot
        Robot.loop();

        /*
         * Show the state and motor powers
         */
        CommonTelemetry.addData("code", "updated");
        CommonTelemetry.addData("Driver Left Stick X value: ", driver.left_stick_x);
        CommonTelemetry.addData("Driver Left Stick Y value: ", driver.left_stick_y);
        CommonTelemetry.addData("Driver Right Stick X value: ", driver.right_stick_y);
        CommonTelemetry.addData("Driver Right Stick X value: ", driver.right_stick_x);
        CommonTelemetry.addData("autonomousDriving", autonomousDriving);

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

    // Lines to the desired pose from the current pose with a linear heading interpolation
    public void lineToPose(Pose desiredPose) {
        Pose currentPose = Robot.follower.getPose();
        double currentHeading = Robot.follower.getHeading();

        Path path = new Path(new BezierLine(currentPose, desiredPose));
        path.setLinearHeadingInterpolation(currentHeading, desiredPose.getHeading());

        // We do NOT touch autonomousDriving here
        Robot.follower.followPath(path);
    }

    public void pedroTeleop() {
        double y = -driver.left_stick_y;
        double x = -driver.left_stick_x;
        double rx = -driver.right_stick_x;

        if (turtleMode.get()) {
            y *= Constants.TURTLE;
            x *= Constants.TURTLE;
            rx *= Constants.TURTLE;
        }

        Robot.follower.setTeleOpDrive(
                y,
                x,
                rx,
                robotCentric // Robot Centric
        );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
