package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(group = "StarterBot")
public class ValueGetter extends LinearOpMode {
    public static double rampPos = 0.0;
    public static double blockerPos = 0.0;
    public static double intakePower = 0.0;
    public static double launcherPower = 0.0;
    public static double feederPower = 0.0;

    @Override
    public void runOpMode() {
        Servo ramp = hardwareMap.get(Servo.class, "ramp");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        DcMotor leftIntake = hardwareMap.get(DcMotorEx.class, "LI");
        DcMotor rightIntake = hardwareMap.get(DcMotorEx.class, "RI");
        DcMotor launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "LF");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "RF");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        CommonTelemetry.init(telemetry);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            ramp.setPosition(rampPos);
            blocker.setPosition(blockerPos);
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);
            launcher.setPower(launcherPower);
            leftFeeder.setPower(feederPower);
            rightFeeder.setPower(feederPower);

            CommonTelemetry.addData("ramp pos", rampPos);
            CommonTelemetry.addData("blocker pos", blockerPos);
            CommonTelemetry.addData("intake power", intakePower);
            CommonTelemetry.addData("launcher power", launcherPower);
            CommonTelemetry.addData("feeder power", feederPower);
            CommonTelemetry.addData("right direction", rightIntake.getDirection());
            CommonTelemetry.addData("left direction", leftIntake.getDirection());
            CommonTelemetry.update();
        }
    }
}
