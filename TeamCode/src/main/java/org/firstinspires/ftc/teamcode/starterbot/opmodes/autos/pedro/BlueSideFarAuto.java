package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import static org.firstinspires.ftc.teamcode.starterbot.Robot.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Side Far Auto (Pedro, Mixed Paths)", group = "Autos")
public class BlueSideFarAuto extends PedroBaseAuto {

    // poses

    // Global start pose
    private final Pose startPose          = new Pose(51.75, 8.75, Math.toRadians(90));

    private final Pose shootPreloadPose   = new Pose(60,   24,   Math.toRadians(118)); // line 0
    private final Pose pickUpMiddlePose   = new Pose(45,   60,   Math.toRadians(180)); // line 1
    private final Pose intakeMiddlePose   = new Pose(17.5, 60,   Math.toRadians(180)); // line 2
    private final Pose openGatePose       = new Pose(16.5, 70.5, Math.toRadians(0));   // line 3
    private final Pose shootMiddlePose    = new Pose(60,   84,   Math.toRadians(131)); // line 4
    private final Pose pickUpTopPose      = new Pose(45,   84,   Math.toRadians(180)); // line 5
    private final Pose intakeTopPose      = new Pose(17.5, 84,   Math.toRadians(180)); // line 6
    private final Pose shootTopPose       = new Pose(60,   84,   Math.toRadians(131)); // line 7
    private final Pose pickUpBottomPose   = new Pose(45,   36,   Math.toRadians(180)); // line 8
    private final Pose intakeBottomPose   = new Pose(16,   36,   Math.toRadians(180)); // line 9
    private final Pose shootBottomPose    = new Pose(60,   24,   Math.toRadians(118)); // line 10
    private final Pose parkPose           = new Pose(20,   70.5, Math.toRadians(0));   // line 11

    // Control points
    private final Pose cpPickUpMiddle1  = new Pose(53.25, 56,    0); // line 1
    private final Pose cpOpenGate1      = new Pose(51,    58,    0); // line 3
    private final Pose cpOpenGate2      = new Pose(59.25, 70,    0); // line 3
    private final Pose cpShootMiddle1   = new Pose(60,    67.25, 0); // line 4
    private final Pose cpShootTop1      = new Pose(49,    92,    0); // line 7
    private final Pose cpPickUpBottom1  = new Pose(60,    34,    0); // line 8
    private final Pose cpShootBottom1   = new Pose(42,    26,    0); // line 10
    private final Pose cpPark1          = new Pose(46,    60,    0); // line 11

    // Paths/PathChains
    // Only pick-up paths are PathChains, everything else is a Path

    private Path      shootPreloadPath;
    private PathChain pickUpMiddleChain;
    private Path      intakeMiddlePath;
    private Path      openGatePath;
    private Path      shootMiddlePath;
    private PathChain pickUpTopChain;
    private Path      intakeTopPath;
    private Path      shootTopPath;
    private PathChain pickUpBottomChain;
    private Path      intakeBottomPath;
    private Path      shootBottomPath;
    private Path      parkPath;

    // abstract class requires override

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        shootPreloadPath = new Path(
                new BezierLine(startPose, shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(118));

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPreloadPose,
                        cpPickUpMiddle1,
                        pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(118),
                        Math.toRadians(180))
                .build();

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        intakeMiddlePath = new Path(
                new BezierLine(pickUpMiddlePose, intakeMiddlePose));
        intakeMiddlePath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(180));

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        openGatePath = new Path(
                new BezierCurve(
                        intakeMiddlePose,
                        cpOpenGate1,
                        cpOpenGate2,
                        openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(0));

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        shootMiddlePath = new Path(
                new BezierCurve(
                        openGatePose,
                        cpShootMiddle1,
                        shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(131));

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootMiddlePose,
                        pickUpTopPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(131),
                        Math.toRadians(180))
                .build();

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        intakeTopPath = new Path(
                new BezierLine(pickUpTopPose, intakeTopPose));
        intakeTopPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(180));

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        shootTopPath = new Path(
                new BezierCurve(
                        intakeTopPose,
                        cpShootTop1,
                        shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(131));

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootTopPose,
                        cpPickUpBottom1,
                        pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(131),
                        Math.toRadians(180))
                .build();

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        intakeBottomPath = new Path(
                new BezierLine(
                        pickUpBottomPose,
                        intakeBottomPose));
        intakeBottomPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(180));

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        shootBottomPath = new Path(
                new BezierCurve(
                        intakeBottomPose,
                        cpShootBottom1,
                        shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(118));

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        parkPath = new Path(
                new BezierCurve(
                        shootBottomPose,
                        cpPark1,
                        parkPose));
        parkPath.setLinearHeadingInterpolation(
                Math.toRadians(118),
                Math.toRadians(0));
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Shoot preload
                follower.followPath(shootPreloadPath);
                setPathState(1);
                break;

            case 1:
                // Pick up middle
                if (!follower.isBusy()) {
                    follower.followPath(pickUpMiddleChain, true); // hold at pickup pose
                    setPathState(2);
                }
                break;

            case 2:
                // Intake middle
                if (!follower.isBusy()) {
                    follower.followPath(intakeMiddlePath);
                    setPathState(3);
                }
                break;

            case 3:
                // Open gate
                if (!follower.isBusy()) {
                    follower.followPath(openGatePath);
                    setPathState(4);
                }
                break;

            case 4:
                // Shoot middle
                if (!follower.isBusy()) {
                    follower.followPath(shootMiddlePath);
                    setPathState(5);
                }
                break;

            case 5:
                // Pick up top
                if (!follower.isBusy()) {
                    follower.followPath(pickUpTopChain, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Intake top
                if (!follower.isBusy()) {
                    follower.followPath(intakeTopPath);
                    setPathState(7);
                }
                break;

            case 7:
                // Shoot top
                if (!follower.isBusy()) {
                    follower.followPath(shootTopPath);
                    setPathState(8);
                }
                break;

            case 8:
                // Pick up bottom
                if (!follower.isBusy()) {
                    follower.followPath(pickUpBottomChain, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Intake bottom
                if (!follower.isBusy()) {
                    follower.followPath(intakeBottomPath);
                    setPathState(10);
                }
                break;

            case 10:
                // Shoot bottom
                if (!follower.isBusy()) {
                    follower.followPath(shootBottomPath);
                    setPathState(11);
                }
                break;

            case 11:
                // Park
                if (!follower.isBusy()) {
                    follower.followPath(parkPath);
                    setPathState(12);
                }
                break;

            case 12:
                // Finished
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                // Idle
                break;
        }
    }
}
