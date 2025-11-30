package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Far Side Red", group = "pedro")
public class FarSideRed extends PedroBaseAuto {
    @Override
    protected Pose getStartPose() {
        return RedFarPoses.startPose;
    }

    @Override
    protected Pose getEndPose() { return RedFarPoses.parkPose; }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        Path shootPreloadPath = new Path(
                new BezierLine(RedFarPoses.startPose, RedFarPoses.shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                RedFarPoses.startPose.getHeading(),           // from startDeg of line 0 / startPose
                RedFarPoses.shootPreloadPose.getHeading());           // endDeg of line 0
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        RedFarPoses.shootPreloadPose,
                        RedFarPoses.cpPickUpMiddle1,
                        RedFarPoses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        RedFarPoses.shootPreloadPose.getHeading(),     // startDeg line 1
                        RedFarPoses.pickUpMiddlePose.getHeading(),      // endDeg line 1
                        0.65)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        Path intakeMiddlePath = new Path(
                new BezierLine(RedFarPoses.pickUpMiddlePose, RedFarPoses.intakeMiddlePose));
        // headingType = constant, startDeg = endDeg = 180
        intakeMiddlePath.setConstantHeadingInterpolation(0);
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        Path openGatePath = new Path(
                new BezierCurve(
                        RedFarPoses.intakeMiddlePose,
                        RedFarPoses.cpOpenGate1,
                        RedFarPoses.cpOpenGate2,
                        RedFarPoses.openGatePose));
        // headingType = linear, startDeg = 0, endDeg = 180
        openGatePath.setLinearHeadingInterpolation(
                RedFarPoses.pickUpMiddlePose.getHeading(),
                RedFarPoses.openGatePose.getHeading(),
                0.8);
        allPaths.add(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        RedFarPoses.openGatePose,
                        RedFarPoses.cpShootMiddle1,
                        RedFarPoses.shootMiddlePose));
        // headingType = linear, startDeg = 180, endDeg = 45
        shootMiddlePath.setLinearHeadingInterpolation(
                RedFarPoses.openGatePose.getHeading(),
                RedFarPoses.shootMiddlePose.getHeading());
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        RedFarPoses.shootMiddlePose,
                        RedFarPoses.pickUpTopPose))
                // headingType = linear, startDeg = 45, endDeg = 0
                .setLinearHeadingInterpolation(
                        RedFarPoses.shootMiddlePose.getHeading(),
                        RedFarPoses.pickUpTopPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpTopChain);

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        Path intakeTopPath = new Path(
                new BezierLine(RedFarPoses.pickUpTopPose, RedFarPoses.intakeTopPose));
        // headingType = constant, 180 deg
        intakeTopPath.setConstantHeadingInterpolation(RedFarPoses.intakeTopPose.getHeading());
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        Path shootTopPath = new Path(
                new BezierCurve(
                        RedFarPoses.intakeTopPose,
                        RedFarPoses.cpShootTop1,
                        RedFarPoses.shootTopPose));
        // headingType = linear, startDeg = 0, endDeg = 45
        shootTopPath.setLinearHeadingInterpolation(
                RedFarPoses.intakeTopPose.getHeading(),
                RedFarPoses.shootTopPose.getHeading());
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);
        allPaths.add(shootTopPath);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        RedFarPoses.shootTopPose,
                        RedFarPoses.cpPickUpBottom1,
                        RedFarPoses.pickUpBottomPose))
                // headingType = linear, startDeg = 45, endDeg = 0
                .setLinearHeadingInterpolation(
                        RedFarPoses.shootTopPose.getHeading(),
                        RedFarPoses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        Path intakeBottomPath = new Path(
                new BezierLine(
                        RedFarPoses.pickUpBottomPose,
                        RedFarPoses.intakeBottomPose));
        // headingType = constant, 180 deg
        intakeBottomPath.setConstantHeadingInterpolation(RedFarPoses.intakeBottomPose.getHeading());
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        Path shootBottomPath = new Path(
                new BezierCurve(
                        RedFarPoses.intakeBottomPose,
                        RedFarPoses.cpShootBottom1,
                        RedFarPoses.shootBottomPose));
        // headingType = linear, startDeg = 180, endDeg = 66
        shootBottomPath.setLinearHeadingInterpolation(
                RedFarPoses.intakeBottomPose.getHeading(),
                RedFarPoses.shootBottomPose.getHeading());
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);
        allPaths.add(shootBottomPath);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        Path parkPath = new Path(
                new BezierCurve(
                        RedFarPoses.shootBottomPose,
                        RedFarPoses.parkPose));
        // headingType = linear, startDeg = 66, endDeg = 180
        parkPath.setLinearHeadingInterpolation(
                RedFarPoses.shootBottomPose.getHeading(),
                RedFarPoses.parkPose.getHeading());
        allPaths.add(parkPath);
    }
}
