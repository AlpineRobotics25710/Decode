package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Far Side Blue", group = "pedro")
public class FarSideBlue extends PedroBaseAuto {
    @Override
    protected Pose getStartPose() {
        return BlueFarPoses.startPose;
    }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        Path shootPreloadPath = new Path(
                new BezierLine(BlueFarPoses.startPose, BlueFarPoses.shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                BlueFarPoses.startPose.getHeading(),
                BlueFarPoses.shootPreloadPose.getHeading());
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueFarPoses.shootPreloadPose,
                        BlueFarPoses.cpPickUpMiddle1,
                        BlueFarPoses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        BlueFarPoses.shootPreloadPose.getHeading(),
                        BlueFarPoses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        Path intakeMiddlePath = new Path(
                new BezierLine(BlueFarPoses.pickUpMiddlePose, BlueFarPoses.intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(BlueFarPoses.intakeMiddlePose.getHeading());
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        Path openGatePath = new Path(
                new BezierCurve(
                        BlueFarPoses.intakeMiddlePose,
                        BlueFarPoses.cpOpenGate1,
                        BlueFarPoses.cpOpenGate2,
                        BlueFarPoses.openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                BlueFarPoses.intakeMiddlePose.getHeading(),
                BlueFarPoses.openGatePose.getHeading(),
                0.8);
        allPaths.add(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        BlueFarPoses.openGatePose,
                        BlueFarPoses.cpShootMiddle1,
                        BlueFarPoses.shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                BlueFarPoses.openGatePose.getHeading(),
                BlueFarPoses.shootMiddlePose.getHeading());
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        BlueFarPoses.shootMiddlePose,
                        BlueFarPoses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        BlueFarPoses.shootMiddlePose.getHeading(),
                        BlueFarPoses.pickUpTopPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpTopChain);

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        Path intakeTopPath = new Path(
                new BezierLine(BlueFarPoses.pickUpTopPose, BlueFarPoses.intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(BlueFarPoses.intakeTopPose.getHeading());
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        Path shootTopPath = new Path(
                new BezierCurve(
                        BlueFarPoses.intakeTopPose,
                        BlueFarPoses.cpShootTop1,
                        BlueFarPoses.shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                BlueFarPoses.intakeTopPose.getHeading(),
                BlueFarPoses.shootTopPose.getHeading());
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueFarPoses.shootTopPose,
                        BlueFarPoses.cpPickUpBottom1,
                        BlueFarPoses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        BlueFarPoses.shootTopPose.getHeading(),
                        BlueFarPoses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        Path intakeBottomPath = new Path(
                new BezierLine(
                        BlueFarPoses.pickUpBottomPose,
                        BlueFarPoses.intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(BlueFarPoses.intakeBottomPose.getHeading());
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        Path shootBottomPath = new Path(
                new BezierCurve(
                        BlueFarPoses.intakeBottomPose,
                        BlueFarPoses.cpShootBottom1,
                        BlueFarPoses.shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                BlueFarPoses.intakeBottomPose.getHeading(),
                BlueFarPoses.shootBottomPose.getHeading());
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        Path parkPath = new Path(
                new BezierCurve(
                        BlueFarPoses.shootBottomPose,
                        BlueFarPoses.cpPark1,
                        BlueFarPoses.parkPose));
        parkPath.setLinearHeadingInterpolation(
                BlueFarPoses.shootBottomPose.getHeading(),
                BlueFarPoses.parkPose.getHeading());
        allPaths.add(parkPath);
    }
}
