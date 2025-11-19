package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Blue Side Close Auto", group = "prod")
public class CloseSideBlue extends PedroBaseAuto {
    @Override
    protected Pose getStartPose() {
        return BlueClosePoses.startPose;
    }

    @Override
    protected void buildPaths() {
        // Line 1: Shoot preload (startPose -> shootPreloadPose), curve, Path
        Path shootPreloadPath = new Path(
                new BezierCurve(
                        BlueClosePoses.startPose,
                        BlueClosePoses.cpShootPreload1,
                        BlueClosePoses.shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                BlueClosePoses.startPose.getHeading(),
                BlueClosePoses.shootPreloadPose.getHeading());
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 2: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueClosePoses.shootPreloadPose,
                        BlueClosePoses.cpPickUpMiddle1,
                        BlueClosePoses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootPreloadPose.getHeading(),
                        BlueClosePoses.pickUpMiddlePose.getHeading())
                .setHeadingConstraint(0.98)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 3: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        Path intakeMiddlePath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpMiddlePose,
                        BlueClosePoses.intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(BlueClosePoses.intakeMiddlePose.getHeading());
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 4: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        Path openGatePath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeMiddlePose,
                        BlueClosePoses.cpOpenGate1,
                        BlueClosePoses.cpOpenGate2,
                        BlueClosePoses.openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeMiddlePose.getHeading(),
                BlueClosePoses.openGatePose.getHeading());
        allPaths.add(openGatePath);

        // Line 5: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        BlueClosePoses.openGatePose,
                        BlueClosePoses.cpShootMiddle1,
                        BlueClosePoses.shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                BlueClosePoses.openGatePose.getHeading(),
                BlueClosePoses.shootMiddlePose.getHeading());
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 6: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        BlueClosePoses.shootMiddlePose,
                        BlueClosePoses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootMiddlePose.getHeading(),
                        BlueClosePoses.pickUpTopPose.getHeading())
                .setHeadingConstraint(0.98)
                .build();
        allPaths.add(pickUpTopChain);

        // Line 7: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        Path intakeTopPath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpTopPose,
                        BlueClosePoses.intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(BlueClosePoses.intakeTopPose.getHeading());
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 8: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        Path shootTopPath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeTopPose,
                        BlueClosePoses.cpShootTop1,
                        BlueClosePoses.shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeTopPose.getHeading(),
                BlueClosePoses.shootTopPose.getHeading());
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 9: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueClosePoses.shootTopPose,
                        BlueClosePoses.cpPickUpBottom1,
                        BlueClosePoses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootTopPose.getHeading(),
                        BlueClosePoses.pickUpBottomPose.getHeading())
                .setHeadingConstraint(0.98)
                .build();
        allPaths.add(pickUpBottomChain);

        // Line 10: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        Path intakeBottomPath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpBottomPose,
                        BlueClosePoses.intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(BlueClosePoses.intakeBottomPose.getHeading());
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 11: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        Path shootBottomPath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeBottomPose,
                        BlueClosePoses.cpShootBottom1,
                        BlueClosePoses.shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeBottomPose.getHeading(),
                BlueClosePoses.shootBottomPose.getHeading());
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 12: Park (shootBottomPose -> parkPose), curve, Path
        Path parkPath = new Path(
                new BezierCurve(
                        BlueClosePoses.shootBottomPose,
                        BlueClosePoses.cpPark1,
                        BlueClosePoses.parkPose));
        parkPath.setLinearHeadingInterpolation(
                BlueClosePoses.shootBottomPose.getHeading(),
                BlueClosePoses.parkPose.getHeading());
        allPaths.add(parkPath);
    }
}
