package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightTester extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose);
            } else {
                telemetry.addData("Limelight", "No targets");
            }

            telemetry.update();
        }
    }
}
