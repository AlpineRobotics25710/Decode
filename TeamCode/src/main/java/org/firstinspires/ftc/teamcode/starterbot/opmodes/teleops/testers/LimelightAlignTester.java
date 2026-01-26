package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

import java.util.List;

@Config
@Configurable
@TeleOp(group = "testers")
public class LimelightAlignTester extends LinearOpMode {
    public static int targetTagId = 20; //blue goal: 20, red goal: 24
    public static double headingTolerance = 1.0;
    public static double turnGain = 0.03;
    public static boolean currentlyAligning = false;

    @Override
    public void runOpMode() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        waitForStart();

        Robot.follower.startTeleopDrive();

        while (!isStopRequested() && opModeIsActive()) {
            limelight.updateRobotOrientation(Math.toDegrees(Robot.follower.getHeading()));

            LLResult llResult = limelight.getLatestResult();
            LLResultTypes.FiducialResult targetTag = null;
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> detectedTags = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : detectedTags) {
                    if (tag.getFiducialId() == targetTagId) {
                        targetTag = tag;
                    }
                }

                if (targetTag != null) {
                    CommonTelemetry.addData("Target x degrees", targetTag.getTargetXDegrees());
                    CommonTelemetry.addData("Target y degrees", targetTag.getTargetYDegrees());
                    CommonTelemetry.addData("target pos robot space", targetTag.getTargetPoseRobotSpace());

                    double headingError = targetTag.getTargetXDegrees();
                    double turn = headingError * turnGain;
                    CommonTelemetry.addData("heading error", headingError);
                    CommonTelemetry.addData("turn power", turn);

                    if (gamepad1.aWasPressed() && !currentlyAligning) {
                        currentlyAligning = true;
                    }

                    if (currentlyAligning) {
                        if (Math.abs(headingError) <= headingTolerance) currentlyAligning = false;

                        Robot.mecanumDrive(0, 0, turn);
                    } else {
                        Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
                    }
                }
            } else {
                CommonTelemetry.addData("Limelight", "No targets found");
            }

            Robot.follower.update();
            CommonTelemetry.draw(Robot.follower);
            CommonTelemetry.addData("pedro heading read deg", Math.toDegrees(Robot.follower.getHeading()));
            CommonTelemetry.update();
        }
    }
}
