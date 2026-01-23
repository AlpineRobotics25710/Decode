package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

@Config
@Configurable
@TeleOp(group = "testers")
public class ShootingTester extends LinearOpMode {

    // 0.3 feeding power, 1275 tps, 0.43 ramp pos
    public static final double MAX_RAMP_DEGREES = 270.0;
    public static final Pose goalPose = new Pose(12, 140);
    public static double rampPosDegrees = 130.95; // 13.5 130.95
    public static double launcherVelocityTicksPerSec = 0;
    public static boolean feedersOn = false;
    public static double feederPower = 0.0;
    public static double kp = 475;
    public static double ki = 10;
    public static double kd = 5;
    public static double ff = 18;

    @Override
    public void runOpMode() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);
        Robot.follower.setStartingPose(new Pose(56.5, 8.75, Math.toRadians(90)));
        Robot.blocker.setPosition(Constants.BLOCKER_OPEN);

        waitForStart();

        Robot.follower.startTeleopDrive();

        while (opModeIsActive() && !isStopRequested()) {
            Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
            Robot.setFeederPower(feedersOn ? feederPower : 0);

            //double rampPos = rampPosDegrees / MAX_RAMP_DEGREES;
            Robot.ramp.setPosition(rampPosDegrees / Constants.MAX_RAMP_DEGREES);
            Robot.ramp2.setPosition(rampPosDegrees / Constants.MAX_RAMP_DEGREES);

            Robot.launcher.setVelocity(launcherVelocityTicksPerSec);
            Robot.launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, ff));

            Robot.follower.update();
            CommonTelemetry.draw(Robot.follower);

            CommonTelemetry.addData("ramp pos", rampPosDegrees / Constants.MAX_RAMP_DEGREES);
            CommonTelemetry.addData("ramp pos deg", rampPosDegrees);
            CommonTelemetry.addData("set launcher velocity (ticks/s)", launcherVelocityTicksPerSec);
            CommonTelemetry.addData("read launcher velocity (ticks/s)", Robot.launcher.getVelocity());
            CommonTelemetry.addData("distance to goal", getDistanceToGoal());
            CommonTelemetry.update();
        }
    }

    public double getDistanceToGoal() {
        Pose currPose = Robot.follower.getPose();
        return currPose.distanceFrom(goalPose);
    }
}
