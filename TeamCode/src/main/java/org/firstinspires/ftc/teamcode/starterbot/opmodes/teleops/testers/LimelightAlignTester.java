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

    @Override
    public void runOpMode() throws InterruptedException {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        waitForStart();

        Robot.follower.startTeleopDrive();

        while (!isStopRequested() && opModeIsActive()) {
            Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);

            limelight.updateRobotOrientation(Math.toDegrees(Robot.follower.getHeading()));

            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> detectedTags = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : detectedTags) {
                    if (tag.getFiducialId() == targetTagId) {
                        CommonTelemetry.addData("Target x degrees", tag.getTargetXDegrees());
                        CommonTelemetry.addData("Target y degrees", tag.getTargetYDegrees());
                        CommonTelemetry.addData("target pos robot space", tag.getTargetPoseRobotSpace());
                    }
                }
            }

            Robot.follower.update();
            CommonTelemetry.draw(Robot.follower);
            CommonTelemetry.addData("pedro heading read deg", Math.toDegrees(Robot.follower.getHeading()));
            CommonTelemetry.update();
        }
    }
}
