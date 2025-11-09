package org.firstinspires.ftc.teamcode.starterbot.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "testers")
@Configurable
public class DrivetrainTester extends LinearOpMode {
    public static double frontLeftPower = 0.0;
    public static double frontRightPower = 0.0;
    public static double backLeftPower = 0.0;
    public static double backRightPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "BL");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            frontRightDrive.setPower(frontRightPower);
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }
    }
}
