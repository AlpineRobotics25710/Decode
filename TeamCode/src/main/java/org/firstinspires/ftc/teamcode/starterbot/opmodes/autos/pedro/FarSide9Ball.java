package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.FarPoses;

@Autonomous(name = "Far Side 9 Ball", group = "pedro")
public class FarSide9Ball extends PedroBaseAuto {
    private FarPoses poses;

    @Override
    protected Pose getStartPose() {
        return poses.startPose;
    }

    @Override
    protected void allianceSetup(Alliance alliance) {
        poses = new FarPoses();
        if (alliance != poses.originalPosesAlliance()) {
            poses.mirror();
        }
    }

    @Override
    protected Pose getEndPose() {
        return poses.parkPose;
    }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootFarPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        PathChain shootPreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.startPose,
                        poses.shootFarPose))
                .setLinearHeadingInterpolation(
                        poses.startPose.getHeading(),
                        poses.shootFarPose.getHeading())
                .build();
        addPath(shootPreloadPath);
        addShot(shootPreloadPath);

        // Line 1: Pick up middle (shootFarPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootFarPose,
                        poses.cpPickUpMiddle1,
                        poses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        poses.shootFarPose.getHeading(),
                        poses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        addPath(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        PathChain intakeMiddlePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.pickUpMiddlePose, poses.intakeMiddlePose))
                .setConstantHeadingInterpolation(poses.intakeMiddlePose.getHeading())
                .build();
        addPath(intakeMiddlePath);
        addIntake(intakeMiddlePath);

        // Line 4: Shoot middle (intakeMiddlePose -> shootFarPose), curve, Path
        PathChain shootMiddlePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeMiddlePose,
                        poses.cpShootMiddle,
                        poses.shootFarPose))
                .setLinearHeadingInterpolation(
                        poses.intakeMiddlePose.getHeading(),
                        poses.shootFarPose.getHeading())
                .build();

        addPath(shootMiddlePath);
        addShot(shootMiddlePath);

        // Line 8: Pick up bottom (shootMiddlePose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootFarPose,
                        poses.cpPickUpBottom,
                        poses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        poses.shootFarPose.getHeading(),
                        poses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        addPath(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        PathChain intakeBottomPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.pickUpBottomPose,
                        poses.intakeBottomPose))
                .setConstantHeadingInterpolation(
                        poses.intakeBottomPose.getHeading())
                .build();
        addPath(intakeBottomPath);
        addIntake(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        PathChain shootBottomPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeBottomPose,
                        poses.cpShootBottom1,
                        poses.shootFarPose))
                .setLinearHeadingInterpolation(
                        poses.intakeBottomPose.getHeading(),
                        poses.shootFarPose.getHeading())
                .build();
        addPath(shootBottomPath);
        addShot(shootBottomPath);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.shootFarPose,
                        poses.parkPose))
                .setLinearHeadingInterpolation(
                        poses.shootFarPose.getHeading(),
                        poses.parkPose.getHeading())
                .build();
        addPath(parkPath);
    }
}
