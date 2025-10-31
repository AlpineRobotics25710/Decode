package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
@TeleOp(group = "testers")
public class AprilTagAlignTest extends BaseTeleOp {
    // Adjust these values to tune the alignment
    public static double turnGain = 0.02;
    public static double headingTolerance = 1.0; // degrees
    public static double goalTagId = 20; // 20 for Blue Alliance goal, 24 for Red Alliance goal
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private IMU imu;

    @Override
    public void initGamepads() {
        driver = gamepad1;
        operator = gamepad2;
    }

    @Override
    public void init() {
        super.init();

        // Initialize the AprilTag processor and Vision Portal
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // Reset yaw at the start of the OpMode
    }

    @Override
    public void loop() {
        super.loop();

        if (driver.a) {
            alignToAprilTag();
        }

        AprilTagDetection goalTag = null;
        if (!aprilTagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                CommonTelemetry.addData("AprilTag Detected: ", detection.id);
                if (detection.id == goalTagId) {
                    goalTag = detection;
                }
            }
        }

        CommonTelemetry.addData("Robot Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        if (goalTag != null) {
            CommonTelemetry.addData("April Tag bearing", goalTag.ftcPose.bearing);
        }

        Robot.loop();
        CommonTelemetry.update();
    }

    private void alignToAprilTag() {
        if (!aprilTagProcessor.getDetections().isEmpty()) {
            // Get the bearing of the red goal AprilTag
            double tagBearing = 0.0;
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                if (detection.id == goalTagId) {
                    tagBearing = detection.ftcPose.bearing;
                }
            }

            // Get the current yaw of the imu
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double imuYaw = orientation.getYaw(AngleUnit.DEGREES);

            // Error in degrees
            double headingError = tagBearing - imuYaw;

            double turn = 0;

            if (Math.abs(headingError) > headingTolerance) {
                // Use a proportional controller to calculate the turn power.
                turn = headingError * turnGain;
            }

            // No forward movement, only turning.
            Robot.arcadeDrive(0, turn);
        }
    }
}
