package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;

@TeleOp(group = "testers")
@Configurable
public class DrivetrainTester extends LinearOpMode {
    public static double frontLeftPower = 0.0;
    public static double frontRightPower = 0.0;
    public static double backLeftPower = 0.0;
    public static double backRightPower = 0.0;

    public static DcMotor.Direction frontLeftDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction frontRightDirection = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction backLeftDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction backRightDirection = DcMotor.Direction.FORWARD;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "BL");

        frontLeftDrive.setDirection(frontLeftDirection);
        backLeftDrive.setDirection(frontRightDirection);
        frontRightDrive.setDirection(backLeftDirection);
        backRightDrive.setDirection(backRightDirection);

        CommonTelemetry.init(telemetry);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            frontLeftDrive.setDirection(frontLeftDirection);
            backLeftDrive.setDirection(frontRightDirection);
            frontRightDrive.setDirection(backLeftDirection);
            backRightDrive.setDirection(backRightDirection);

            frontRightDrive.setPower(frontRightPower);
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            CommonTelemetry.addData("Front left direction", frontLeftDrive.getDirection());
            CommonTelemetry.addData("Front right direction", frontRightDrive.getDirection());
            CommonTelemetry.addData("Back left direction", backLeftDrive.getDirection());
            CommonTelemetry.addData("Back right direction", backRightDrive.getDirection());

            CommonTelemetry.addData("Front left power", frontLeftDrive.getPower());
            CommonTelemetry.addData("Front right power", frontRightDrive.getPower());
            CommonTelemetry.addData("Back left power", backLeftDrive.getPower());
            CommonTelemetry.addData("Back right power", backRightDrive.getPower());

            CommonTelemetry.update();
        }
    }
}
