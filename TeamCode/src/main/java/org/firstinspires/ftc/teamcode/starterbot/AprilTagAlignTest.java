package org.firstinspires.ftc.teamcode.starterbot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

        // Initialize the AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder().addProcessor(aprilTagProcessor).build();

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
            telemetry.addData("AprilTag Detected", aprilTagProcessor.getDetections().get(0).id);
        }
        telemetry.addData("Robot Yaw", "%.2f degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Robot.loop();
        CommonTelemetry.update();
    }

    private void alignToAprilTag() {
        if (!aprilTagProcessor.getDetections().isEmpty()) {
            AprilTagDetection detection = aprilTagProcessor.getDetections().get(0);

            // To be "square" with the tag, we need to correct for both the angle *to* the tag (bearing)
            // and the tag's own rotation relative to the robot (yaw).
            // The error is the combination of these two angles.
            double headingError = detection.ftcPose.yaw - detection.ftcPose.bearing;

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
