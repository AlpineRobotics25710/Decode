package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;

@Configurable
@Config
@TeleOp(group = "testers")
public class ValueGetter extends LinearOpMode {
    public static double rampPos = 0.0;
    public static double blockerPos = 0.1;
    public static double intakePower = 0.0;
    public static double launcherVelocity = 0.0; // this is in ticks/second
    public static double feederPower = 0.0;

    @Override
    public void runOpMode() {
        Servo ramp = hardwareMap.get(Servo.class, "ramp");
        Servo ramp2 = hardwareMap.get(Servo.class,"ramp2");

        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        DcMotor leftIntake = hardwareMap.get(DcMotorEx.class, "LI");
        DcMotor rightIntake = hardwareMap.get(DcMotorEx.class, "RI");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "LF");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "RF");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        ramp2.setDirection(Servo.Direction.REVERSE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(475, 10, 5, 18));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        CommonTelemetry.init(telemetry);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            ramp.setPosition(rampPos);
            ramp2.setPosition(rampPos);
            blocker.setPosition(blockerPos);
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);
            launcher.setVelocity(launcherVelocity);
            leftFeeder.setPower(feederPower);
            rightFeeder.setPower(feederPower);

            CommonTelemetry.addData("ramp pos", rampPos);
            CommonTelemetry.addData("ramp pos 2 raw", ramp2.getPosition());
            CommonTelemetry.addData("blocker pos", blockerPos);
            CommonTelemetry.addData("intake power", intakePower);
            CommonTelemetry.addData("launcher velocity", launcherVelocity);
            CommonTelemetry.addData("feeder power", feederPower);
            CommonTelemetry.addData("right direction", rightIntake.getDirection());
            CommonTelemetry.addData("left direction", leftIntake.getDirection());
            CommonTelemetry.update();
        }
    }
}
