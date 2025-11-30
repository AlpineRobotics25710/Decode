package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Far Side Auto (Alliance Select)", group = "pedro")
public class FarSideAuto extends PedroAllianceAuto {

    @Override
    protected Pose getStartPose() {
        // BlueFarPoses.startPose mirrored automatically on RED
        return alliancePose(BlueFarPoses.startPose);
    }

    @Override
    protected void buildPaths() {
        // 0: Shoot preload
        Path shootPreloadPath = new Path(
                new BezierLine(
                        alliancePose(BlueFarPoses.startPose),
                        alliancePose(BlueFarPoses.shootPreloadPose)
                )
        );
        shootPreloadPath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.startPose).getHeading(),
                alliancePose(BlueFarPoses.shootPreloadPose).getHeading(),
                0.65
        );
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);

        // 1: Pick up middle (PathChain)
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(BlueFarPoses.shootPreloadPose),
                        alliancePose(BlueFarPoses.cpPickUpMiddle1),
                        alliancePose(BlueFarPoses.pickUpMiddlePose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueFarPoses.shootPreloadPose).getHeading(),
                        alliancePose(BlueFarPoses.pickUpMiddlePose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpMiddleChain);

        // 2: Intake middle
        Path intakeMiddlePath = new Path(
                new BezierLine(
                        alliancePose(BlueFarPoses.pickUpMiddlePose),
                        alliancePose(BlueFarPoses.intakeMiddlePose)
                )
        );
        intakeMiddlePath.setConstantHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeMiddlePose).getHeading()
        );
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);

        // 3: Open gate
        Path openGatePath = new Path(
                new BezierCurve(
                        alliancePose(BlueFarPoses.intakeMiddlePose),
                        alliancePose(BlueFarPoses.cpOpenGate1),
                        alliancePose(BlueFarPoses.cpOpenGate2),
                        alliancePose(BlueFarPoses.openGatePose)
                )
        );
        openGatePath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeMiddlePose).getHeading(),
                alliancePose(BlueFarPoses.openGatePose).getHeading(),
                0.8
        );
        allPaths.add(openGatePath);

        // 4: Shoot middle
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        alliancePose(BlueFarPoses.openGatePose),
                        alliancePose(BlueFarPoses.cpShootMiddle1),
                        alliancePose(BlueFarPoses.shootMiddlePose)
                )
        );
        shootMiddlePath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.openGatePose).getHeading(),
                alliancePose(BlueFarPoses.shootMiddlePose).getHeading(),
                0.65
        );
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // 5: Pick up top (PathChain)
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        alliancePose(BlueFarPoses.shootMiddlePose),
                        alliancePose(BlueFarPoses.pickUpTopPose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueFarPoses.shootMiddlePose).getHeading(),
                        alliancePose(BlueFarPoses.pickUpTopPose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpTopChain);

        // 6: Intake top
        Path intakeTopPath = new Path(
                new BezierLine(
                        alliancePose(BlueFarPoses.pickUpTopPose),
                        alliancePose(BlueFarPoses.intakeTopPose)
                )
        );
        intakeTopPath.setConstantHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeTopPose).getHeading()
        );
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);

        // 7: Shoot top
        Path shootTopPath = new Path(
                new BezierCurve(
                        alliancePose(BlueFarPoses.intakeTopPose),
                        alliancePose(BlueFarPoses.cpShootTop1),
                        alliancePose(BlueFarPoses.shootTopPose)
                )
        );
        shootTopPath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeTopPose).getHeading(),
                alliancePose(BlueFarPoses.shootTopPose).getHeading(),
                0.65
        );
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);

        // 8: Pick up bottom (PathChain)
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(BlueFarPoses.shootTopPose),
                        alliancePose(BlueFarPoses.cpPickUpBottom1),
                        alliancePose(BlueFarPoses.pickUpBottomPose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueFarPoses.shootTopPose).getHeading(),
                        alliancePose(BlueFarPoses.pickUpBottomPose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpBottomChain);

        // 9: Intake bottom
        Path intakeBottomPath = new Path(
                new BezierLine(
                        alliancePose(BlueFarPoses.pickUpBottomPose),
                        alliancePose(BlueFarPoses.intakeBottomPose)
                )
        );
        intakeBottomPath.setConstantHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeBottomPose).getHeading()
        );
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);

        // 10: Shoot bottom
        Path shootBottomPath = new Path(
                new BezierCurve(
                        alliancePose(BlueFarPoses.intakeBottomPose),
                        alliancePose(BlueFarPoses.cpShootBottom1),
                        alliancePose(BlueFarPoses.shootBottomPose)
                )
        );
        shootBottomPath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.intakeBottomPose).getHeading(),
                alliancePose(BlueFarPoses.shootBottomPose).getHeading(),
                0.65
        );
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);

        // 11: Park
        Path parkPath = new Path(
                new BezierCurve(
                        alliancePose(BlueFarPoses.shootBottomPose),
                        alliancePose(BlueFarPoses.cpPark1),
                        alliancePose(BlueFarPoses.parkPose)
                )
        );
        parkPath.setLinearHeadingInterpolation(
                alliancePose(BlueFarPoses.shootBottomPose).getHeading(),
                alliancePose(BlueFarPoses.parkPose).getHeading()
        );
        allPaths.add(parkPath);
    }
}
