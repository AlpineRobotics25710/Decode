package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

@Configurable
public class FarPoses implements AutonomousPoses {
    // Shooting angles
    public double FAR_SHOOTING_ANGLE;
    public double CLOSE_SHOOTING_ANGLE;

    // Poses
    public Pose startPose;
    public Pose shootFarPose;
    public Pose pickUpMiddlePose;
    public Pose intakeMiddlePose;
    public Pose openGatePose;
    public Pose shootClosePose;
    public Pose pickUpTopPose;
    public Pose intakeTopPose;
    public Pose pickUpBottomPose;
    public Pose intakeBottomPose;
    public Pose parkPose;

    // Control points
    public Pose cpPickUpMiddle1;
    public Pose cpOpenGate1;
    public Pose cpOpenGate2;
    public Pose cpShootMiddle1;
    public Pose cpShootTop1;
    public Pose cpPickUpBottom1;
    public Pose cpShootBottom1;
    public Pose cpPark1;

    public FarPoses() {
        FAR_SHOOTING_ANGLE = Math.toRadians(111.5);
        CLOSE_SHOOTING_ANGLE = Math.toRadians(135);

        // Poses
        startPose = new Pose(56.5, 8.75, Math.toRadians(90));
        shootFarPose = new Pose(54.5, 15.5, FAR_SHOOTING_ANGLE);
        pickUpMiddlePose = new Pose(50, 58.5, Math.toRadians(180));
        intakeMiddlePose = new Pose(16, 58.5, Math.toRadians(180));
        openGatePose = new Pose(15, 70.5, Math.toRadians(0));
        shootClosePose = new Pose(53, 91, CLOSE_SHOOTING_ANGLE);
        pickUpTopPose = new Pose(50, 83, Math.toRadians(180));
        intakeTopPose = new Pose(15.5, 83, Math.toRadians(180));
        pickUpBottomPose = new Pose(50, 35, Math.toRadians(180));
        intakeBottomPose = new Pose(14.5, 35, Math.toRadians(180));
        parkPose = new Pose(53, 35, Math.toRadians(90));

        // Control points
        cpPickUpMiddle1 = new Pose(53.25, 56);
        cpOpenGate1 = new Pose(51, 58);
        cpOpenGate2 = new Pose(58.5, 64);
        cpShootMiddle1 = new Pose(60, 67.25);
        cpShootTop1 = new Pose(49, 92);
        cpPickUpBottom1 = new Pose(64, 58);
        cpShootBottom1 = new Pose(42, 26);
        //cpPark1 = new Pose(46, 60);
    }
    
    @Override
    public void mirror() {
        startPose = startPose.mirror();
        shootFarPose = shootFarPose.mirror();
        pickUpMiddlePose = pickUpMiddlePose.mirror();
        intakeMiddlePose = intakeMiddlePose.mirror();
        openGatePose = openGatePose.mirror();
        shootClosePose = shootClosePose.mirror();
        pickUpTopPose = pickUpTopPose.mirror();
        intakeTopPose = intakeTopPose.mirror();
        shootClosePose = shootClosePose.mirror();
        pickUpBottomPose = pickUpBottomPose.mirror();
        intakeBottomPose = intakeBottomPose.mirror();
        shootFarPose = shootFarPose.mirror();
        parkPose = parkPose.mirror();
        cpPickUpMiddle1 = cpPickUpMiddle1.mirror();
        cpOpenGate1 = cpOpenGate1.mirror();
        cpOpenGate2 = cpOpenGate2.mirror();
        cpShootMiddle1 = cpShootMiddle1.mirror();
        cpShootTop1 = cpShootTop1.mirror();
        cpPickUpBottom1 = cpPickUpBottom1.mirror();
        cpShootBottom1 = cpShootBottom1.mirror();
        //cpPark1 = cpPark1.mirror();

        // Update headings
        CLOSE_SHOOTING_ANGLE = (Math.PI) - CLOSE_SHOOTING_ANGLE;
        FAR_SHOOTING_ANGLE = (Math.PI) - FAR_SHOOTING_ANGLE;
    }
    
    @Override
    public Alliance originalPosesAlliance() {
        return Alliance.BLUE;
    }
}
