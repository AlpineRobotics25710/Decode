package org.firstinspires.ftc.teamcode.starterbot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagAlignTest extends BaseTeleOp {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private IMU imu;

    // Adjust these values to tune the alignment
    public static double TURN_GAIN = 0.02;
    public static double HEADING_TOLERANCE = 1.0; // degrees

    @Override
    public void initGamepads() {
        driver = gamepad1;
        operator = gamepad2;
    }

    @Override
    public void init() {
        super.init();

        // Initialize the AprilTag processor and Vision Portal
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
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

        if (!aprilTagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections())
                CommonTelemetry.addData("AprilTag Detected: ", detection.id);
        }
        CommonTelemetry.addData("Robot Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Robot.loop();
        CommonTelemetry.update();
    }

    private void alignToAprilTag() {
        if (!aprilTagProcessor.getDetections().isEmpty()) {
            // Get the bearing of the red goal AprilTag
            double tagBearing = 0.0;
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                if (detection.id == 24) {
                    tagBearing = detection.ftcPose.bearing;
                }
            }

            // Get the current yaw of the imu
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double imuYaw = orientation.getYaw(AngleUnit.DEGREES);

            // Error in degrees
            double headingError = tagBearing - imuYaw;

            double turn = 0;

            if (Math.abs(headingError) > HEADING_TOLERANCE) {
                // Use a proportional controller to calculate the turn power.
                turn = headingError * TURN_GAIN;
            }

            // No forward movement, only turning.
            Robot.arcadeDrive(0, turn);
        }
    }
}
