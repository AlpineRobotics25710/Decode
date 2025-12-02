package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.ClosePoses;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.FarPoses;

@Autonomous(name = "Blue Side Close Auto", group = "pedro")
public class CloseSideAuto extends PedroBaseAuto {
    @Override
    protected Pose getStartPose() {
        return ClosePoses.startPose;
    }

    @Override
    protected void allianceSetup(Alliance alliance) {
        if (alliance != new ClosePoses().originalPosesAlliance()) {
            new ClosePoses().mirror();
        }
    }

    @Override
    protected Pose getEndPose() { return BlueClosePoses.parkPose; }

    @Override
    protected void buildPaths() {
        // Line 1: Shoot preload (startPose -> shootPreloadPose), curve, Path
        Path shootPreloadPath = new Path(
                new BezierLine(
                        ClosePoses.startPose,
                        ClosePoses.shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                ClosePoses.startPose.getHeading(),
                ClosePoses.shootPreloadPose.getHeading());
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ClosePoses.shootPreloadPose,
                        ClosePoses.cpPickUpMiddle1,
                        ClosePoses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        ClosePoses.shootPreloadPose.getHeading(),
                        ClosePoses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpMiddleChain);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        Path intakeMiddlePath = new Path(
                new BezierLine(ClosePoses.pickUpMiddlePose, ClosePoses.intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(ClosePoses.intakeMiddlePose.getHeading());
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        Path openGatePath = new Path(
                new BezierCurve(
                        ClosePoses.intakeMiddlePose,
                        ClosePoses.cpOpenGate1,
                        ClosePoses.cpOpenGate2,
                        ClosePoses.openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                ClosePoses.intakeMiddlePose.getHeading(),
                ClosePoses.openGatePose.getHeading(),
                0.8);
        allPaths.add(openGatePath);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        ClosePoses.openGatePose,
                        ClosePoses.cpShootMiddle1,
                        ClosePoses.shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                ClosePoses.openGatePose.getHeading(),
                ClosePoses.shootMiddlePose.getHeading(),
                0.65);
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        ClosePoses.shootMiddlePose,
                        ClosePoses.pickUpTopPose))
                .setLinearHeadingInterpolation(
                        ClosePoses.shootMiddlePose.getHeading(),
                        ClosePoses.pickUpTopPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpTopChain);

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        Path intakeTopPath = new Path(
                new BezierLine(ClosePoses.pickUpTopPose, ClosePoses.intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(ClosePoses.intakeTopPose.getHeading());
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        Path shootTopPath = new Path(
                new BezierCurve(
                        ClosePoses.intakeTopPose,
                        ClosePoses.cpShootTop1,
                        ClosePoses.shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                ClosePoses.intakeTopPose.getHeading(),
                ClosePoses.shootTopPose.getHeading(),
                0.65);
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ClosePoses.shootTopPose,
                        ClosePoses.cpPickUpBottom1,
                        ClosePoses.pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        ClosePoses.shootTopPose.getHeading(),
                        ClosePoses.pickUpBottomPose.getHeading(),
                        0.65)
                .build();
        allPaths.add(pickUpBottomChain);

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        Path intakeBottomPath = new Path(
                new BezierLine(
                        ClosePoses.pickUpBottomPose,
                        ClosePoses.intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(ClosePoses.intakeBottomPose.getHeading());
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        Path shootBottomPath = new Path(
                new BezierCurve(
                        ClosePoses.intakeBottomPose,
                        ClosePoses.cpShootBottom1,
                        ClosePoses.shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                ClosePoses.intakeBottomPose.getHeading(),
                ClosePoses.shootBottomPose.getHeading(),
                0.65);
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        Path parkPath = new Path(
                new BezierLine(
                        ClosePoses.shootBottomPose,
                        ClosePoses.parkPose));
        parkPath.setLinearHeadingInterpolation(
                ClosePoses.shootBottomPose.getHeading(),
                ClosePoses.parkPose.getHeading());
        allPaths.add(parkPath);
    }
}
