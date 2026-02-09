package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
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
    public static boolean currentlyAligning = false;
    public static double kP = 0.0095;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.03;
    private static final PIDFController alignPIDF = new PIDFController(new PIDFCoefficients(kP, kI, kD, 0));

    @Override
    public void runOpMode() {
        CommonTelemetry.init(telemetry);
        Robot.init(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        Robot.follower.setStartingPose(new Pose(56.5, 8.75, Math.toRadians(90)));

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
                    double headingError = -targetTag.getTargetXDegrees();
                    alignPIDF.updateError(headingError);
                    double turn = alignPIDF.run();
                    CommonTelemetry.addData("turn power", turn);

                    // minimum motor power is heading still exists
                    if (Math.abs(headingError) > headingTolerance) {
                        turn += (Math.signum(turn) * kF);
                    } else {
                        // Small corrective hold, NOT zero
                        turn *= 0.2; // could cause problems, if there are problems maybe lower or remove this
                    }

                    // clamp output
                    turn = Math.min(1.0, Math.max(-1.0, turn));

                    if (currentlyAligning) {
                        Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, turn);
                    }
                }
            } else {
                CommonTelemetry.addData("Limelight", "No targets found");
                if (currentlyAligning) {
                    Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, 0);
                }
            }

            if (gamepad1.aWasPressed()) {
                currentlyAligning = !currentlyAligning;
                alignPIDF.reset();
            }

            if (gamepad1.bWasPressed()) {
                Robot.queueLaunch();
            }

            if (gamepad1.xWasPressed()) {
                Robot.switchRampState();
            }

            if (gamepad1.yWasPressed()) {
                Robot.switchBlockerState();
            }

            if (!currentlyAligning) {
                Robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
            }

            alignPIDF.setP(kP);
            alignPIDF.setI(kI);
            alignPIDF.setD(kD);

            Robot.follower.update();
            CommonTelemetry.draw(Robot.follower);
            CommonTelemetry.addData("currentlyAligning", currentlyAligning);
            CommonTelemetry.addData("pedro heading read deg", Math.toDegrees(Robot.follower.getHeading()));
            Robot.loop();
            CommonTelemetry.update();
        }
    }
}
