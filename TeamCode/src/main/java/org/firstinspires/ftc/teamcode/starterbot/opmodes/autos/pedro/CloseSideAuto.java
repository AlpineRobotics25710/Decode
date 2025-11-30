package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Close Side Auto (Alliance Select)", group = "pedro")
public class CloseSideAuto extends PedroAllianceAuto {

    @Override
    protected Pose getStartPose() {
        return alliancePose(BlueClosePoses.startPose);
    }

    @Override
    protected void buildPaths() {

        // Line 0: Shoot Preload
        Path shootPreloadPath = new Path(
                new BezierLine(
                        alliancePose(BlueClosePoses.startPose),
                        alliancePose(BlueClosePoses.shootPreloadPose)
                )
        );
        shootPreloadPath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.startPose).getHeading(),
                alliancePose(BlueClosePoses.shootPreloadPose).getHeading(),
                0.65
        );
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);


        // Line 1: Pickup Middle
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(BlueClosePoses.shootPreloadPose),
                        alliancePose(BlueClosePoses.cpPickUpMiddle1),
                        alliancePose(BlueClosePoses.pickUpMiddlePose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueClosePoses.shootPreloadPose).getHeading(),
                        alliancePose(BlueClosePoses.pickUpMiddlePose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpMiddleChain);


        // Line 2: Intake Middle
        Path intakeMiddlePath = new Path(
                new BezierLine(
                        alliancePose(BlueClosePoses.pickUpMiddlePose),
                        alliancePose(BlueClosePoses.intakeMiddlePose)
                )
        );
        intakeMiddlePath.setConstantHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeMiddlePose).getHeading()
        );
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);


        // Line 3: Open Gate
        Path openGatePath = new Path(
                new BezierCurve(
                        alliancePose(BlueClosePoses.intakeMiddlePose),
                        alliancePose(BlueClosePoses.cpOpenGate1),
                        alliancePose(BlueClosePoses.cpOpenGate2),
                        alliancePose(BlueClosePoses.openGatePose)
                )
        );
        openGatePath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeMiddlePose).getHeading(),
                alliancePose(BlueClosePoses.openGatePose).getHeading(),
                0.8
        );
        allPaths.add(openGatePath);


        // Line 4: Shoot Middle
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        alliancePose(BlueClosePoses.openGatePose),
                        alliancePose(BlueClosePoses.cpShootMiddle1),
                        alliancePose(BlueClosePoses.shootMiddlePose)
                )
        );
        shootMiddlePath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.openGatePose).getHeading(),
                alliancePose(BlueClosePoses.shootMiddlePose).getHeading(),
                0.65
        );
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);


        // Line 5: Pick Up Top
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        alliancePose(BlueClosePoses.shootMiddlePose),
                        alliancePose(BlueClosePoses.pickUpTopPose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueClosePoses.shootMiddlePose).getHeading(),
                        alliancePose(BlueClosePoses.pickUpTopPose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpTopChain);


        // Line 6: Intake Top
        Path intakeTopPath = new Path(
                new BezierLine(
                        alliancePose(BlueClosePoses.pickUpTopPose),
                        alliancePose(BlueClosePoses.intakeTopPose)
                )
        );
        intakeTopPath.setConstantHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeTopPose).getHeading()
        );
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);


        // Line 7: Shoot Top
        Path shootTopPath = new Path(
                new BezierCurve(
                        alliancePose(BlueClosePoses.intakeTopPose),
                        alliancePose(BlueClosePoses.cpShootTop1),
                        alliancePose(BlueClosePoses.shootTopPose)
                )
        );
        shootTopPath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeTopPose).getHeading(),
                alliancePose(BlueClosePoses.shootTopPose).getHeading(),
                0.65
        );
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);


        // Line 8: Pick Up Bottom
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        alliancePose(BlueClosePoses.shootTopPose),
                        alliancePose(BlueClosePoses.cpPickUpBottom1),
                        alliancePose(BlueClosePoses.pickUpBottomPose)
                ))
                .setLinearHeadingInterpolation(
                        alliancePose(BlueClosePoses.shootTopPose).getHeading(),
                        alliancePose(BlueClosePoses.pickUpBottomPose).getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpBottomChain);


        // Line 9: Intake Bottom
        Path intakeBottomPath = new Path(
                new BezierLine(
                        alliancePose(BlueClosePoses.pickUpBottomPose),
                        alliancePose(BlueClosePoses.intakeBottomPose)
                )
        );
        intakeBottomPath.setConstantHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeBottomPose).getHeading()
        );
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);


        // Line 10: Shoot Bottom
        Path shootBottomPath = new Path(
                new BezierCurve(
                        alliancePose(BlueClosePoses.intakeBottomPose),
                        alliancePose(BlueClosePoses.cpShootBottom1),
                        alliancePose(BlueClosePoses.shootBottomPose)
                )
        );
        shootBottomPath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.intakeBottomPose).getHeading(),
                alliancePose(BlueClosePoses.shootBottomPose).getHeading(),
                0.65
        );
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);


        // Line 11: Park
        Path parkPath = new Path(
                new BezierCurve(
                        alliancePose(BlueClosePoses.shootBottomPose),
                        alliancePose(BlueClosePoses.cpPark1),
                        alliancePose(BlueClosePoses.parkPose)
                )
        );
        parkPath.setLinearHeadingInterpolation(
                alliancePose(BlueClosePoses.shootBottomPose).getHeading(),
                alliancePose(BlueClosePoses.parkPose).getHeading()
        );
        allPaths.add(parkPath);
    }
}
