package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

@Configurable
public class ClosePoses implements AutonomousPoses {
    public double CLOSE_SHOOTING_ANGLE;

    // Global start pose (Blue coordinates)
    public Pose startPose;

    public Pose shootPreloadPose;
    public Pose pickUpMiddlePose;
    public Pose intakeMiddlePose;
    public Pose openGatePose;
    public Pose shootMiddlePose;
    public Pose pickUpTopPose;
    public Pose intakeTopPose;
    public Pose shootTopPose;
    public Pose pickUpBottomPose;
    public Pose intakeBottomPose;
    public Pose shootBottomPose;
    public Pose parkPose;

    // Control points (Blue coordinates)
    public Pose cpPickUpMiddle1;
    public Pose cpOpenGate1;
    public Pose cpOpenGate2;
    public Pose cpShootMiddle1;
    public Pose cpShootTop1;
    public Pose cpPickUpBottom1;
    public Pose cpShootBottom1;

    public ClosePoses() {
        CLOSE_SHOOTING_ANGLE = Math.toRadians(136);

        startPose = new Pose(21, 122, Math.toRadians(143.5));
        shootPreloadPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);
        pickUpMiddlePose = new Pose(50, 58.5, Math.toRadians(180));
        intakeMiddlePose = new Pose(14, 58.5, Math.toRadians(180));
        openGatePose = new Pose(15, 70.5, Math.toRadians(0));
        shootMiddlePose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);
        pickUpTopPose = new Pose(50, 83, Math.toRadians(180));
        intakeTopPose = new Pose(16.5, 83, Math.toRadians(180));
        shootTopPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);
        pickUpBottomPose = new Pose(50, 35, Math.toRadians(180));
        intakeBottomPose = new Pose(12.5, 35, Math.toRadians(180));
        shootBottomPose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);
        parkPose = new Pose(35, 80, Math.toRadians(180));
        cpPickUpMiddle1 = new Pose(58, 70);
        cpOpenGate1 = new Pose(51, 58);
        cpOpenGate2 = new Pose(58.5, 64);
        cpShootMiddle1 = new Pose(60, 67.25);
        cpShootTop1 = new Pose(49, 92);
        cpPickUpBottom1 = new Pose(66, 57);
        cpShootBottom1 = new Pose(53, 48);
    }

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

        // Update heading
        CLOSE_SHOOTING_ANGLE = (Math.PI) - CLOSE_SHOOTING_ANGLE;
    }
    
    @Override
    public Alliance originalPosesAlliance() {
        return Alliance.BLUE;
    }
}
