package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.25119)// input robot mass
            .forwardZeroPowerAcceleration(-30.499437451291932) // need to test
            .lateralZeroPowerAcceleration(-59.74626160865985) // need to test
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.002, 0.005)) // need to tune
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0, 0.02)) //need to tune
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.65, 0, 0, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.25, 0, 0, 0, 0.022)) // need to tune
            .useSecondaryHeadingPIDF(true)
            .centripetalScaling(0.0005); // need to test

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.35, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.69173016886073) // need to test
            .yVelocity(53.80969959168923); // need to test

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-33.3) // values have been updated
            .strafePodX(-111.7) // values have been updated
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); // need to test for reversed

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
