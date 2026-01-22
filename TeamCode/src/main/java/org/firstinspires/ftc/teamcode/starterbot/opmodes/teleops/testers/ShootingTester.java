package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

@TeleOp(group = "testers")
@Config
@Configurable
public class ShootingTester extends LinearOpMode {
    public static final double MAX_RAMP_DEGREES = 270.0;
    public static final Pose goalPose = new Pose(12, 140);
    public static double rampPosDegrees = 130.95; // 13.5 130.95
    public static double launcherVelocityTicksPerSec = 1.5;
    public static boolean feedersOn = false;
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

        MultipleTelemetry mt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Robot.setFeederPower(feedersOn ? Constants.FEEDER_POWER : 0);

            double rampPos = rampPosDegrees / MAX_RAMP_DEGREES;
            Robot.ramp.setPosition(rampPos);

            Robot.launcher.setVelocity(launcherVelocityTicksPerSec);
            Robot.launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, ff));

            Robot.follower.update();
            CommonTelemetry.draw(Robot.follower);

            mt.addData("ramp pos", rampPos);
            mt.addData("ramp pos deg", rampPosDegrees);
            mt.addData("set launcher velocity (ticks/s)", launcherVelocityTicksPerSec);
            mt.addData("read launcher velocity (ticks/s)", Robot.launcher.getVelocity());
            mt.addData("distance to goal", getDistanceToGoal());
            mt.update();
        }
    }

    public double getDistanceToGoal() {
        Pose currPose = Robot.follower.getPose();
        return currPose.distanceFrom(goalPose);
    }
}
