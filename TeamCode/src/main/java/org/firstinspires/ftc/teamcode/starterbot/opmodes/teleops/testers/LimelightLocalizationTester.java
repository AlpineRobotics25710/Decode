package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Locale;

@Config
@Configurable
@TeleOp
public class LimelightLocalizationTester extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu = hardwareMap.get(IMU.class, "imu");


        pinpoint.setOffsets(-111.7, -33.3, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56.5, 8.75, AngleUnit.DEGREES, 90));
        pinpoint.initialize();

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
                )
        );

        limelight.pipelineSwitch(2);
        limelight.start();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(yaw);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Limelight localization data
                Pose3D llPose = result.getBotpose_MT2();
                Position llPos = llPose.getPosition().toUnit(DistanceUnit.INCH);
                double llx = llPos.x;
                double lly = llPos.y;
                double llh = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addLine("------ Limelight Data (Inches) ------");
                telemetry.addData("Robot X", String.format(Locale.US, "%.3f", llx));
                telemetry.addData("Robot Y", String.format(Locale.US, "%.3f", lly));
                telemetry.addData("Robot Heading", String.format(Locale.US, "%.3f", llh));

                // Pinpoint localization data
                Pose2D ppPose = pinpoint.getPosition();
                double ppx = ppPose.getX(DistanceUnit.INCH);
                double ppy = ppPose.getY(DistanceUnit.INCH);
                double pph = ppPose.getHeading(AngleUnit.DEGREES);

                telemetry.addLine("------ Pinpoint Data (Inches) ------");
                telemetry.addData("Robot X", String.format(Locale.US, "%.3f", ppx));
                telemetry.addData("Robot Y", String.format(Locale.US, "%.3f", ppy));
                telemetry.addData("Robot Heading", String.format(Locale.US, "%.3f", pph));
            } else {
                telemetry.addData("Limelight", "No targets visible");
            }

            pinpoint.update();
            telemetry.update();
        }
    }
}
