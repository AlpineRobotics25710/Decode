package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.ClosePoses;

@Autonomous(name = "Close Side Auto", group = "pedro")
public class CloseSideAuto extends PedroBaseAuto {
    private ClosePoses poses;

    @Override
    protected Pose getStartPose() {
        return poses.startPose;
    }

    @Override
    protected void allianceSetup(Alliance alliance) {
        poses = new ClosePoses();
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
        // Line 1: Shoot preload (startPose -> shootPreloadPose), curve, Path
        PathChain shootPreloadPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                poses.startPose,
                                poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.startPose.getHeading(),
                        poses.shootPose.getHeading())
                .build();
        allPaths.add(shootPreloadPath);
        addShot(shootPreloadPath);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootPose,
                        poses.cpPickUpMiddle1,
                        poses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        PathChain intakeMiddlePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.pickUpMiddlePose, poses.intakeMiddlePose))
                .setConstantHeadingInterpolation(poses.intakeMiddlePose.getHeading())
                .build();
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        PathChain openGatePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                poses.intakeMiddlePose,
                                poses.cpOpenGate1,
                                poses.cpOpenGate2,
                                poses.openGatePose))
                .setLinearHeadingInterpolation(
                        poses.intakeMiddlePose.getHeading(),
                        poses.openGatePose.getHeading(),
                        0.8)
                .build();
        allPaths.add(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        PathChain shootMiddlePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                poses.openGatePose,
                                poses.cpShootMiddle1,
                                poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.openGatePose.getHeading(),
                        poses.shootPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(shootMiddlePath);
        addShot(shootMiddlePath);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.shootPose,
                        poses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.pickUpTopPose.getHeading(),
                        0.65)
                .build();
        addPath(pickUpTopChain);

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        PathChain intakeTopPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.pickUpTopPose, poses.intakeTopPose))
                .setConstantHeadingInterpolation(poses.intakeTopPose.getHeading())
                .build();
        addPath(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        PathChain shootTopPath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                poses.intakeTopPose,
                                poses.cpShootTop1,
                                poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.intakeTopPose.getHeading(),
                        poses.shootPose.getHeading(),
                        0.65)
                .build();
        addPath(shootTopPath);
        addShot(shootTopPath);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootPose,
                        poses.cpPickUpBottom1,
                        poses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        addPath(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        PathChain intakeBottomPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                poses.pickUpBottomPose,
                                poses.intakeBottomPose))
                .setConstantHeadingInterpolation(poses.intakeBottomPose.getHeading())
                .build();
        addPath(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        PathChain shootBottomPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeBottomPose,
                        poses.cpShootBottom1,
                        poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.intakeBottomPose.getHeading(),
                        poses.shootPose.getHeading(),
                        0.65)
                .build();
        addPath(shootBottomPath);
        addShot(shootBottomPath);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        PathChain parkPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                poses.shootPose,
                                poses.parkPose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.parkPose.getHeading())
                .build();
        addPath(parkPath);
    }
}
