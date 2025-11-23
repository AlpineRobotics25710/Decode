package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
class BlueFarPoses {
    // BLUE-SIDE TEMPLATE POSES

    // blue
    public static double FAR_SHOOTING_ANGLE = Math.toRadians(120);
    public static double CLOSE_SHOOTING_ANGLE = Math.toRadians(136);

    // Global start pose (Blue coordinates)
    public static final Pose startPose = new Pose(56.5, 8.75, Math.toRadians(90));

    public static final Pose shootPreloadPose = new Pose(54.5, 15.5, FAR_SHOOTING_ANGLE);                         // line 0
    public static final Pose pickUpMiddlePose = new Pose(50, 58.5, Math.toRadians(180));                        // line 1
    public static final Pose intakeMiddlePose = new Pose(16, 58.5, Math.toRadians(180));                        // line 2
    public static final Pose openGatePose = new Pose(15, 70.5, Math.toRadians(0));                          // line 3
    public static final Pose shootMiddlePose = new Pose(56, 84, CLOSE_SHOOTING_ANGLE); // line 4
    public static final Pose pickUpTopPose = new Pose(50, 83, Math.toRadians(180));                        // line 5
    public static final Pose intakeTopPose = new Pose(15.5, 83, Math.toRadians(180));                        // line 6
    public static final Pose shootTopPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                       // line 7
    public static final Pose pickUpBottomPose = new Pose(50, 35, Math.toRadians(180));                        // line 8
    public static final Pose intakeBottomPose = new Pose(14.5, 35, Math.toRadians(180));                        // line 9
    public static final Pose shootBottomPose = new Pose(54.5, 12, FAR_SHOOTING_ANGLE);                         // line 10
    public static final Pose parkPose = new Pose(20, 70.5, Math.toRadians(0));                          // line 11

    // Control points (Blue coordinates)
    public static final Pose cpPickUpMiddle1 = new Pose(53.25, 56, 0); // line 1
    public static final Pose cpOpenGate1 = new Pose(51, 58, 0); // line 3
    public static final Pose cpOpenGate2 = new Pose(58.5, 64, 0); // line 3
    public static final Pose cpShootMiddle1 = new Pose(60, 67.25, 0); // line 4
    public static final Pose cpShootTop1 = new Pose(49, 92, 0); // line 7
    public static final Pose cpPickUpBottom1 = new Pose(58, 35.5, 0); // line 8
    public static final Pose cpShootBottom1 = new Pose(42, 26, 0); // line 10
    public static final Pose cpPark1 = new Pose(46, 60, 0); // line 11
}

@Configurable
class RedFarPoses {
    // red
    public static double FIRST_FAR_SHOOTING_ANGLE = Math.toRadians(59.5);
    public static double LAST_FAR_SHOOTING_ANGLE = Math.toRadians(63);
    public static double CLOSE_SHOOTING_ANGLE = Math.toRadians(43);

    // POSES (RED)

    // Global start pose
    public static final Pose startPose = new Pose(87.5, 8.75, Math.toRadians(90));

    public static final Pose shootPreloadPose = new Pose(90, 15.5, FIRST_FAR_SHOOTING_ANGLE);   // line 0: Shoot preload
    public static final Pose pickUpMiddlePose = new Pose(94, 58.5, Math.toRadians(0));    // line 1: Pick up middle
    public static final Pose intakeMiddlePose = new Pose(126.5, 58.5, Math.toRadians(0));  // line 2: Intake middle
    public static final Pose openGatePose = new Pose(129, 70.5, Math.toRadians(180));  // line 3: Open gate
    public static final Pose shootMiddlePose = new Pose(88, 88, CLOSE_SHOOTING_ANGLE);   // line 4: Shoot middle
    public static final Pose pickUpTopPose = new Pose(94, 83, Math.toRadians(0));    // line 5: Pick up top
    public static final Pose intakeTopPose = new Pose(128, 83, Math.toRadians(0));  // line 6: Intake top
    public static final Pose shootTopPose = new Pose(88, 88, CLOSE_SHOOTING_ANGLE);   // line 7: Shoot top
    public static final Pose pickUpBottomPose = new Pose(94, 35, Math.toRadians(0));    // line 8: Pick up bottom
    public static final Pose intakeBottomPose = new Pose(128, 35, Math.toRadians(0));  // line 9: Intake bottom
    public static final Pose shootBottomPose = new Pose(90, 15.5, LAST_FAR_SHOOTING_ANGLE);   // line 10: Shoot bottom
    public static final Pose parkPose = new Pose(116, 70, Math.toRadians(180));  // line 11: Park

    // Control points from RedSideFarAuto-Decode.pp
    public static final Pose cpPickUpMiddle1 = new Pose(90.75, 56, 0);   // line 1
    public static final Pose cpOpenGate1 = new Pose(93, 58, 0);   // line 3
    public static final Pose cpOpenGate2 = new Pose(85.5, 64, 0);   // line 3
    public static final Pose cpShootMiddle1 = new Pose(85, 57.5, 0);   // line 4
    public static final Pose cpShootTop1 = new Pose(95, 92, 0);   // line 7
    public static final Pose cpPickUpBottom1 = new Pose(86, 35.5, 0);   // line 8
    public static final Pose cpShootBottom1 = new Pose(107.25, 17.5, 0);   // line 10
    public static final Pose cpPark1 = new Pose(98, 60, 0);   // line 11
}

@Configurable
class BlueClosePoses {
    public static double CLOSE_SHOOTING_ANGLE = Math.toRadians(134);

    // Global start pose (Blue coordinates)
    public static final Pose startPose = new Pose(21, 122, Math.toRadians(143.5));

    public static final Pose shootPreloadPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                         // line 0
    public static final Pose pickUpMiddlePose = new Pose(50, 58.5, Math.toRadians(180));                        // line 1
    public static final Pose intakeMiddlePose = new Pose(16, 58.5, Math.toRadians(180));                        // line 2
    public static final Pose openGatePose = new Pose(15, 70.5, Math.toRadians(0));                          // line 3
    public static final Pose shootMiddlePose = new Pose(56, 84, CLOSE_SHOOTING_ANGLE); // line 4
    public static final Pose pickUpTopPose = new Pose(50, 83, Math.toRadians(180));                        // line 5
    public static final Pose intakeTopPose = new Pose(15.5, 83, Math.toRadians(180));                        // line 6
    public static final Pose shootTopPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                       // line 7
    public static final Pose pickUpBottomPose = new Pose(50, 35, Math.toRadians(180));                        // line 8
    public static final Pose intakeBottomPose = new Pose(14.5, 35, Math.toRadians(180));                        // line 9
    public static final Pose shootBottomPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);                         // line 10
    public static final Pose parkPose = new Pose(35, 80, Math.toRadians(180));                          // line 11

    // Control points (Blue coordinates)
    public static final Pose cpPickUpMiddle1 = new Pose(58, 70); // line 1
    public static final Pose cpOpenGate1 = new Pose(51, 58); // line 3
    public static final Pose cpOpenGate2 = new Pose(58.5, 64); // line 3
    public static final Pose cpShootMiddle1 = new Pose(60, 67.25); // line 4
    public static final Pose cpShootTop1 = new Pose(49, 92); // line 7
    public static final Pose cpPickUpBottom1 = new Pose(66, 57); // line 8
    public static final Pose cpShootBottom1 = new Pose(53, 48); // line 10
}
