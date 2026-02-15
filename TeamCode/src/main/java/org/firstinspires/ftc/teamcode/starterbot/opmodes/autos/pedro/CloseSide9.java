package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.ClosePoses;

@Autonomous(name = "Close Side 9 Ball", group = "pedro")
public class CloseSide9 extends PedroBaseAuto {
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
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        PathChain shootPreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.startPose,
                        poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.startPose.getHeading(),
                        poses.shootPose.getHeading())
                .build();
        addPath(shootPreloadPath);
        addShot(shootPreloadPath);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        PathChain pickUpMiddlePath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootPose,
                        poses.cpPickUpMiddle1,
                        poses.pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.pickUpMiddlePose.getHeading(),
                        0.65)
                .build();
        addPath(pickUpMiddlePath);

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        PathChain intakeMiddlePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.pickUpMiddlePose, poses.intakeMiddlePose))
                .setConstantHeadingInterpolation(poses.intakeMiddlePose.getHeading())
                .build();
        addPath(intakeMiddlePath);
        addIntake(intakeMiddlePath);

        // Line 3: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        PathChain shootMiddlePath = follower.pathBuilder()
                .addPath( new BezierCurve(
                        poses.intakeMiddlePose,
                        poses.cpShootMiddle1,
                        poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.intakeMiddlePose.getHeading(),
                        poses.shootPose.getHeading())
                .build();
        addPath(shootMiddlePath);
        addShot(shootMiddlePath);

        // Line 4: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
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

        // Line 5: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        PathChain intakeTopPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.pickUpTopPose, poses.intakeTopPose))
                .setConstantHeadingInterpolation(poses.intakeTopPose.getHeading())
                .build();
        addPath(intakeTopPath);
        addIntake(intakeTopPath);

        // Line 6: Shoot top (intakeTopPose -> shootPose), curve, Path
        PathChain shootTopPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.intakeTopPose,
                        poses.cpShootTop1,
                        poses.shootPose))
                .setLinearHeadingInterpolation(
                        poses.intakeTopPose.getHeading(),
                        poses.shootPose.getHeading())
                .build();
        addPath(shootTopPath);
        addShot(shootTopPath);

        // Line 7: Park (shootPose -> parkPose), curve, Path
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poses.shootPose,
                        poses.cpParkPose,
                        poses.parkPose))
                .setLinearHeadingInterpolation(
                        poses.shootPose.getHeading(),
                        poses.parkPose.getHeading())
                .build();
        addPath(parkPath);
    }
}
