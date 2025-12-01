package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

@Configurable
public class ClosePoses implements AutonomousPoses {
    public static double CLOSE_SHOOTING_ANGLE = Math.toRadians(134);

    // Global start pose (Blue coordinates)
    public static Pose startPose = new Pose(21, 122, Math.toRadians(143.5));

    public static Pose shootPreloadPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                         // line 0
    public static Pose pickUpMiddlePose = new Pose(50, 58.5, Math.toRadians(180));                        // line 1
    public static Pose intakeMiddlePose = new Pose(16, 58.5, Math.toRadians(180));                        // line 2
    public static Pose openGatePose = new Pose(15, 70.5, Math.toRadians(0));                          // line 3
    public static Pose shootMiddlePose = new Pose(56, 84, CLOSE_SHOOTING_ANGLE); // line 4
    public static Pose pickUpTopPose = new Pose(50, 83, Math.toRadians(180));                        // line 5
    public static Pose intakeTopPose = new Pose(15.5, 83, Math.toRadians(180));                        // line 6
    public static Pose shootTopPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                       // line 7
    public static Pose pickUpBottomPose = new Pose(50, 35, Math.toRadians(180));                        // line 8
    public static Pose intakeBottomPose = new Pose(14.5, 35, Math.toRadians(180));                        // line 9
    public static Pose shootBottomPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                         // line 10
    public static Pose parkPose = new Pose(35, 80, Math.toRadians(180));                          // line 11

    // Control points (Blue coordinates)
    public static Pose cpPickUpMiddle1 = new Pose(58, 70); // line 1
    public static Pose cpOpenGate1 = new Pose(51, 58); // line 3
    public static Pose cpOpenGate2 = new Pose(58.5, 64); // line 3
    public static Pose cpShootMiddle1 = new Pose(60, 67.25); // line 4
    public static Pose cpShootTop1 = new Pose(49, 92); // line 7
    public static Pose cpPickUpBottom1 = new Pose(66, 57); // line 8
    public static Pose cpShootBottom1 = new Pose(53, 48); // line 10

    @Override
    public void mirror() {
        startPose = startPose.mirror();
        shootPreloadPose = shootPreloadPose.mirror();
        pickUpMiddlePose = pickUpMiddlePose.mirror();
        intakeMiddlePose = intakeMiddlePose.mirror();
        openGatePose = openGatePose.mirror();
        shootMiddlePose = shootMiddlePose.mirror();
        pickUpTopPose = pickUpTopPose.mirror();
        intakeTopPose = intakeTopPose.mirror();
        shootTopPose = shootTopPose.mirror();
        pickUpBottomPose = pickUpBottomPose.mirror();
        intakeBottomPose = intakeBottomPose.mirror();
        shootBottomPose = shootBottomPose.mirror();
        parkPose = parkPose.mirror();
        cpPickUpMiddle1 = cpPickUpMiddle1.mirror();
        cpOpenGate1 = cpOpenGate1.mirror();
        cpOpenGate2 = cpOpenGate2.mirror();
        cpShootMiddle1 = cpShootMiddle1.mirror();
        cpShootTop1 = cpShootTop1.mirror();
        cpPickUpBottom1 = cpPickUpBottom1.mirror();
        cpShootBottom1 = cpShootBottom1.mirror();
    }

    @Override
    public Alliance originalPosesAlliance() {
        return Alliance.BLUE;
    }
}
