package org.firstinspires.ftc.teamcode.starterbot;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(group = "testers")
public class AprilTagAlignTest extends BaseTeleOp {
    // Adjust these values to tune the alignment
    public static double turnGain = 0.095;
    public static double headingTolerance = 8.5; // degrees
    public static double goalTagId = 20; // 20 for Blue Alliance goal, 24 for Red Alliance goal
    public static boolean alignRequested = false;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 3, 0, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void initGamepads() {
        driver = gamepad1;
        operator = gamepad2;
    }

    @Override
    public void init() {
        super.init();

        // Initialize the AprilTag processor and Vision Portal
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        if (driver.dpadLeftWasPressed()) {
            goalTagId = 20;
        } else if (driver.dpadRightWasPressed()) {
            goalTagId = 24;
        }

        telemetryAprilTag();
        setManualExposure(Constants.EXPOSURE_MS, Constants.GAIN);
        CommonTelemetry.update();
    }

    @Override
    public void loop() {
        if (driver.aWasPressed()) {
            alignRequested = !alignRequested;
        }

        if (driver.dpadLeftWasPressed()) {
            goalTagId = 20;
        } else if (driver.dpadRightWasPressed()) {
            goalTagId = 24;
        }

        setManualExposure(Constants.EXPOSURE_MS, Constants.GAIN);

        if (alignRequested) {
            alignToAprilTag();
        }

        telemetryAprilTag();

        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.id == goalTagId) {
                CommonTelemetry.addData("Target april tag bearing: ", detection.ftcPose.bearing);
                break;
            }
        }

        CommonTelemetry.addData("Target tag", goalTagId);
        CommonTelemetry.addData("Align requested", alignRequested);
        super.loop();
    }

    private void alignToAprilTag() {
        AprilTagDetection targetTag = null;
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.id == goalTagId) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag == null) {
            Robot.arcadeDrive(0, 0);
            return;
        }

        // Error in degrees
        double headingError = targetTag.ftcPose.bearing;
        CommonTelemetry.addData("heading error", headingError);

        if (headingError > 0) {
            headingError += 5;
        } else {
            headingError -= 5;
        }

        double turn = 0;

        if (Math.abs(headingError) > headingTolerance) {
            // Use a proportional controller to calculate the turn power.
            turn = headingError * turnGain;
        } else {
            alignRequested = false;
        }

        // No forward movement, only turning.
        Robot.arcadeDrive(0, turn);
    }

    private void setManualExposure(double exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                //Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            //Thread.sleep(50);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        //Thread.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        //Thread.sleep(20);
        telemetry.addData("Camera", "Ready");
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        CommonTelemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                CommonTelemetry.debug(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    CommonTelemetry.debug(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    CommonTelemetry.debug(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                CommonTelemetry.debug(String.format("\n==== (ID %d) Unknown", detection.id));
                CommonTelemetry.debug(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        CommonTelemetry.debug("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        CommonTelemetry.debug("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }
}
