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
    public Pose cpShootMiddle12;
    public Pose cpShootMiddle;
    public Pose cpShootTop1;
    public Pose cpPickUpBottom12;
    public Pose cpPickUpBottom;
    public Pose cpShootBottom1;

    public FarPoses() {
        FAR_SHOOTING_ANGLE = Math.toRadians(109);
        CLOSE_SHOOTING_ANGLE = Math.toRadians(136);

        // Poses
        startPose = new Pose(56.5, 8.75, Math.toRadians(90));
        shootFarPose = new Pose(54.5, 15.5, FAR_SHOOTING_ANGLE);
        shootClosePose = new Pose(56, 88, CLOSE_SHOOTING_ANGLE);

        pickUpMiddlePose = new Pose(48, 59, Math.toRadians(180));
        cpPickUpMiddle1 = new Pose(53.25, 56);
        intakeMiddlePose = new Pose(16, 59, Math.toRadians(180));
        cpShootMiddle12 = new Pose(60, 67.25); // for 12 ball only lowk ignore...
        cpShootMiddle = new Pose (55, 51);

        pickUpBottomPose = new Pose(48, 35, Math.toRadians(180));
        cpPickUpBottom12 = new Pose(64, 58); // for 12 ball only lowk ignore...
        cpPickUpBottom = new Pose(54, 30);
        intakeBottomPose = new Pose(16, 35, Math.toRadians(180));
        cpShootBottom1 = new Pose(42, 26);

        parkPose = new Pose(40, 18, Math.toRadians(109));

        // additional poses for 12 ball
        openGatePose = new Pose(15, 70.5, Math.toRadians(0));
        cpOpenGate1 = new Pose(51, 58);
        cpOpenGate2 = new Pose(58.5, 64);

        pickUpTopPose = new Pose(50, 83, Math.toRadians(180));
        intakeTopPose = new Pose(15.5, 83, Math.toRadians(180));
        cpShootTop1 = new Pose(49, 92);
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
        pickUpBottomPose = pickUpBottomPose.mirror();
        intakeBottomPose = intakeBottomPose.mirror();
        parkPose = parkPose.mirror();
        cpPickUpMiddle1 = cpPickUpMiddle1.mirror();
        cpOpenGate1 = cpOpenGate1.mirror();
        cpOpenGate2 = cpOpenGate2.mirror();
        cpShootMiddle12 = cpShootMiddle12.mirror();
        cpShootTop1 = cpShootTop1.mirror();
        cpPickUpBottom12 = cpPickUpBottom12.mirror();
        cpShootBottom1 = cpShootBottom1.mirror();

        // Update headings
        CLOSE_SHOOTING_ANGLE = (Math.PI) - CLOSE_SHOOTING_ANGLE;
        FAR_SHOOTING_ANGLE = (Math.PI) - FAR_SHOOTING_ANGLE;
    }
    
    @Override
    public Alliance originalPosesAlliance() {
        return Alliance.BLUE;
    }
}
