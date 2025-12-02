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

@Autonomous(name = "Far Side Blue", group = "pedro")
public class FarSideAuto extends PedroBaseAuto {
    @Override
    protected Pose getStartPose() {
        return FarPoses.startPose;
    }

    @Override
    protected void allianceSetup(Alliance alliance) {
        if (alliance != new FarPoses().originalPosesAlliance()) {
            new FarPoses().mirror();
        }
    }

    @Override
    protected Pose getEndPose() { return FarPoses.parkPose; }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        Path shootPreloadPath = new Path(
                new BezierLine(FarPoses.startPose, FarPoses.shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                FarPoses.startPose.getHeading(),
                FarPoses.shootPreloadPose.getHeading());
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        FarPoses.shootPreloadPose,
                        FarPoses.cpPickUpMiddle1,
                        FarPoses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        FarPoses.shootPreloadPose.getHeading(),
                        FarPoses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        Path intakeMiddlePath = new Path(
                new BezierLine(FarPoses.pickUpMiddlePose, FarPoses.intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(FarPoses.intakeMiddlePose.getHeading());
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        Path openGatePath = new Path(
                new BezierCurve(
                        FarPoses.intakeMiddlePose,
                        FarPoses.cpOpenGate1,
                        FarPoses.cpOpenGate2,
                        FarPoses.openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                FarPoses.intakeMiddlePose.getHeading(),
                FarPoses.openGatePose.getHeading(),
                0.8);
        allPaths.add(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        FarPoses.openGatePose,
                        FarPoses.cpShootMiddle1,
                        FarPoses.shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                FarPoses.openGatePose.getHeading(),
                FarPoses.shootMiddlePose.getHeading());
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        FarPoses.shootMiddlePose,
                        FarPoses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        FarPoses.shootMiddlePose.getHeading(),
                        FarPoses.pickUpTopPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpTopChain);

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        Path intakeTopPath = new Path(
                new BezierLine(FarPoses.pickUpTopPose, FarPoses.intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(FarPoses.intakeTopPose.getHeading());
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        Path shootTopPath = new Path(
                new BezierCurve(
                        FarPoses.intakeTopPose,
                        FarPoses.cpShootTop1,
                        FarPoses.shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                FarPoses.intakeTopPose.getHeading(),
                FarPoses.shootTopPose.getHeading());
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        FarPoses.shootTopPose,
                        FarPoses.cpPickUpBottom1,
                        FarPoses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        FarPoses.shootTopPose.getHeading(),
                        FarPoses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        Path intakeBottomPath = new Path(
                new BezierLine(
                        FarPoses.pickUpBottomPose,
                        FarPoses.intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(FarPoses.intakeBottomPose.getHeading());
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        Path shootBottomPath = new Path(
                new BezierCurve(
                        FarPoses.intakeBottomPose,
                        FarPoses.cpShootBottom1,
                        FarPoses.shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                FarPoses.intakeBottomPose.getHeading(),
                FarPoses.shootBottomPose.getHeading());
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        Path parkPath = new Path(
                new BezierCurve(
                        FarPoses.shootBottomPose,
                        FarPoses.parkPose));
        parkPath.setLinearHeadingInterpolation(
                FarPoses.shootBottomPose.getHeading(),
                FarPoses.parkPose.getHeading());
        allPaths.add(parkPath);
    }
}
