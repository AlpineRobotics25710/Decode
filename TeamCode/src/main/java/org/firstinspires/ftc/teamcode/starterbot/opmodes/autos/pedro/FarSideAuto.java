package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.FarPoses;

@Autonomous(name = "Far Side Auto", group = "pedro")
public class FarSideAuto extends PedroBaseAuto {
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
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        PathChain shootPreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.startPose,
                        poses.shootPreloadPose))
                .setLinearHeadingInterpolation(
                        poses.startPose.getHeading(),
                        poses.shootPreloadPose.getHeading())
                .build();
        addPath(shootPreloadPath);
        addShot(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootPreloadPose,
                        poses.cpPickUpMiddle1,
                        poses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        poses.shootPreloadPose.getHeading(),
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

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        PathChain openGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeMiddlePose,
                        poses.cpOpenGate1,
                        poses.cpOpenGate2,
                        poses.openGatePose))
                .setLinearHeadingInterpolation(
                        poses.intakeMiddlePose.getHeading(),
                        poses.openGatePose.getHeading(),
                        0.8)
                .build();
        addPath(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        PathChain shootMiddlePath = follower.pathBuilder()
                .addPath( new BezierCurve(
                        poses.openGatePose,
                        poses.cpShootMiddle1,
                        poses.shootMiddlePose))
                .setLinearHeadingInterpolation(
                        poses.openGatePose.getHeading(),
                        poses.shootMiddlePose.getHeading())
                .build();

        addPath(shootMiddlePath);
        addShot(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.shootMiddlePose,
                        poses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        poses.shootMiddlePose.getHeading(),
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
        addIntake(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        PathChain shootTopPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeTopPose,
                        poses.cpShootTop1,
                        poses.shootTopPose))
                .setLinearHeadingInterpolation(
                        poses.intakeTopPose.getHeading(),
                        poses.shootTopPose.getHeading())
                .build();
        addPath(shootTopPath);
        addShot(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootTopPose,
                        poses.cpPickUpBottom1,
                        poses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        poses.shootTopPose.getHeading(),
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
                        poses.shootBottomPose))
                .setLinearHeadingInterpolation(
                        poses.intakeBottomPose.getHeading(),
                        poses.shootBottomPose.getHeading())
                .build();
        addPath(shootBottomPath);
        addShot(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.shootBottomPose,
                        poses.parkPose))
                .setLinearHeadingInterpolation(
                        poses.shootBottomPose.getHeading(),
                        poses.parkPose.getHeading())
                .build();
        addPath(parkPath);
    }
}
