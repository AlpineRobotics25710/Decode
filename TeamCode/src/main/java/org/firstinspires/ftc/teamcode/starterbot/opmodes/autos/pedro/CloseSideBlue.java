package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Blue Side Close Auto", group = "pedro")
public class CloseSideBlue extends PedroBaseAuto {

    @Override
    protected Pose getStartPose() {
        return BlueClosePoses.startPose;
    }

    @Override
    protected void buildPaths() {


        // Line 0: Shoot Preload
        Path shootPreloadPath = new Path(
                new BezierLine(
                        BlueClosePoses.startPose,
                        BlueClosePoses.shootPreloadPose
                )
        );
        shootPreloadPath.setLinearHeadingInterpolation(
                BlueClosePoses.startPose.getHeading(),
                BlueClosePoses.shootPreloadPose.getHeading(),
                0.65
        );
        allPaths.add(shootPreloadPath);
        shotNeeded.put(shootPreloadPath, Constants.LAUNCHER_FAR_VELOCITY);


        // Line 1: Pick Up Middle
        PathChain pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueClosePoses.shootPreloadPose,
                        BlueClosePoses.cpPickUpMiddle1,
                        BlueClosePoses.pickUpMiddlePose
                ))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootPreloadPose.getHeading(),
                        BlueClosePoses.pickUpMiddlePose.getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpMiddleChain);


        // Line 2: Intake Middle
        Path intakeMiddlePath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpMiddlePose,
                        BlueClosePoses.intakeMiddlePose
                )
        );
        intakeMiddlePath.setConstantHeadingInterpolation(
                BlueClosePoses.intakeMiddlePose.getHeading()
        );
        allPaths.add(intakeMiddlePath);
        intakeNeeded.add(intakeMiddlePath);


        // Line 3: Open Gate
        Path openGatePath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeMiddlePose,
                        BlueClosePoses.cpOpenGate1,
                        BlueClosePoses.cpOpenGate2,
                        BlueClosePoses.openGatePose
                )
        );
        openGatePath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeMiddlePose.getHeading(),
                BlueClosePoses.openGatePose.getHeading(),
                0.8
        );
        allPaths.add(openGatePath);


        // Line 4: Shoot Middle
        Path shootMiddlePath = new Path(
                new BezierCurve(
                        BlueClosePoses.openGatePose,
                        BlueClosePoses.cpShootMiddle1,
                        BlueClosePoses.shootMiddlePose
                )
        );
        shootMiddlePath.setLinearHeadingInterpolation(
                BlueClosePoses.openGatePose.getHeading(),
                BlueClosePoses.shootMiddlePose.getHeading(),
                0.65
        );
        allPaths.add(shootMiddlePath);
        shotNeeded.put(shootMiddlePath, Constants.LAUNCHER_CLOSE_VELOCITY);


        // Line 5: Pick Up Top
        PathChain pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        BlueClosePoses.shootMiddlePose,
                        BlueClosePoses.pickUpTopPose
                ))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootMiddlePose.getHeading(),
                        BlueClosePoses.pickUpTopPose.getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpTopChain);


        // Line 6: Intake Top
        Path intakeTopPath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpTopPose,
                        BlueClosePoses.intakeTopPose
                )
        );
        intakeTopPath.setConstantHeadingInterpolation(
                BlueClosePoses.intakeTopPose.getHeading()
        );
        allPaths.add(intakeTopPath);
        intakeNeeded.add(intakeTopPath);


        // Line 7: Shoot Top
        Path shootTopPath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeTopPose,
                        BlueClosePoses.cpShootTop1,
                        BlueClosePoses.shootTopPose
                )
        );
        shootTopPath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeTopPose.getHeading(),
                BlueClosePoses.shootTopPose.getHeading(),
                0.65
        );
        allPaths.add(shootTopPath);
        shotNeeded.put(shootTopPath, Constants.LAUNCHER_CLOSE_VELOCITY);


        // Line 8: Pick Up Bottom
        PathChain pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        BlueClosePoses.shootTopPose,
                        BlueClosePoses.cpPickUpBottom1,
                        BlueClosePoses.pickUpBottomPose
                ))
                .setLinearHeadingInterpolation(
                        BlueClosePoses.shootTopPose.getHeading(),
                        BlueClosePoses.pickUpBottomPose.getHeading(),
                        0.65
                )
                .build();
        allPaths.add(pickUpBottomChain);


        // Line 9: Intake Bottom
        Path intakeBottomPath = new Path(
                new BezierLine(
                        BlueClosePoses.pickUpBottomPose,
                        BlueClosePoses.intakeBottomPose
                )
        );
        intakeBottomPath.setConstantHeadingInterpolation(
                BlueClosePoses.intakeBottomPose.getHeading()
        );
        allPaths.add(intakeBottomPath);
        intakeNeeded.add(intakeBottomPath);


        // Line 10: Shoot Bottom
        Path shootBottomPath = new Path(
                new BezierCurve(
                        BlueClosePoses.intakeBottomPose,
                        BlueClosePoses.cpShootBottom1,
                        BlueClosePoses.shootBottomPose
                )
        );
        shootBottomPath.setLinearHeadingInterpolation(
                BlueClosePoses.intakeBottomPose.getHeading(),
                BlueClosePoses.shootBottomPose.getHeading(),
                0.65
        );
        allPaths.add(shootBottomPath);
        shotNeeded.put(shootBottomPath, Constants.LAUNCHER_FAR_VELOCITY);


        // Line 11: Park
        Path parkPath = new Path(
                new BezierCurve(
                        BlueClosePoses.shootBottomPose,
                        BlueClosePoses.cpPark1,
                        BlueClosePoses.parkPose
                )
        );
        parkPath.setLinearHeadingInterpolation(
                BlueClosePoses.shootBottomPose.getHeading(),
                BlueClosePoses.parkPose.getHeading()
        );
        allPaths.add(parkPath);
    }
}
