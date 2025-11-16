package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Far Side Auto (Alliance Select)", group = "Pedro Autos")
public class FarSideAuto extends PedroBaseAuto {

    // BLUE-SIDE TEMPLATE POSES

    // Global start pose (BLUE coordinates)
    private final Pose startPoseBlue        = new Pose(51.75, 8.75, Math.toRadians(90));

    private final Pose shootPreloadPoseBlue = new Pose(60,   12,   FAR_SHOOTING_ANGLE);                         // line 0
    private final Pose pickUpMiddlePoseBlue = new Pose(50,   58.5, Math.toRadians(180));                        // line 1
    private final Pose intakeMiddlePoseBlue = new Pose(16,   58.5, Math.toRadians(180));                        // line 2
    private final Pose openGatePoseBlue     = new Pose(15,   70.5, Math.toRadians(0));                          // line 3
    private final Pose shootMiddlePoseBlue  = new Pose(60,   84,   CLOSE_SHOOTING_ANGLE + Math.toRadians(2.5)); // line 4
    private final Pose pickUpTopPoseBlue    = new Pose(50,   83,   Math.toRadians(180));                        // line 5
    private final Pose intakeTopPoseBlue    = new Pose(15.5, 83,   Math.toRadians(180));                        // line 6
    private final Pose shootTopPoseBlue     = new Pose(58.5, 84,   CLOSE_SHOOTING_ANGLE);                       // line 7
    private final Pose pickUpBottomPoseBlue = new Pose(50,   35,   Math.toRadians(180));                        // line 8
    private final Pose intakeBottomPoseBlue = new Pose(14.5, 35,   Math.toRadians(180));                        // line 9
    private final Pose shootBottomPoseBlue  = new Pose(60,   12,   FAR_SHOOTING_ANGLE);                         // line 10
    private final Pose parkPoseBlue         = new Pose(20,   70.5, Math.toRadians(0));                          // line 11

    // Control points (BLUE coordinates)
    private final Pose cpPickUpMiddle1Blue  = new Pose(53.25, 56,    0); // line 1
    private final Pose cpOpenGate1Blue      = new Pose(51,    58,    0); // line 3
    private final Pose cpOpenGate2Blue      = new Pose(59.25, 70,    0); // line 3
    private final Pose cpShootMiddle1Blue   = new Pose(60,    67.25, 0); // line 4
    private final Pose cpShootTop1Blue      = new Pose(49,    92,    0); // line 7
    private final Pose cpPickUpBottom1Blue  = new Pose(60,    34,    0); // line 8
    private final Pose cpShootBottom1Blue   = new Pose(42,    26,    0); // line 10
    private final Pose cpPark1Blue          = new Pose(46,    60,    0); // line 11

    // EFFECTIVE POSES (MIRRORED IF RED)
    private Pose startPose;
    private Pose shootPreloadPose;
    private Pose pickUpMiddlePose;
    private Pose intakeMiddlePose;
    private Pose openGatePose;
    private Pose shootMiddlePose;
    private Pose pickUpTopPose;
    private Pose intakeTopPose;
    private Pose shootTopPose;
    private Pose pickUpBottomPose;
    private Pose intakeBottomPose;
    private Pose shootBottomPose;
    private Pose parkPose;

    private Pose cpPickUpMiddle1;
    private Pose cpOpenGate1;
    private Pose cpOpenGate2;
    private Pose cpShootMiddle1;
    private Pose cpShootTop1;
    private Pose cpPickUpBottom1;
    private Pose cpShootBottom1;
    private Pose cpPark1;

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

    // Alliance / mirroring helpers

    /** Mirror a BLUE-side pose to RED if needed, based on PedroBaseAuto's alliance detection. */
    private Pose mirrorIfRed(Pose bluePose) {
        return isRedAlliance() ? bluePose.mirror() : bluePose;
    }

    /** Build effective poses/control points from the BLUE templates, mirroring if we're Red. */
    private void buildEffectivePoses() {
        // Main poses
        startPose        = mirrorIfRed(startPoseBlue);
        shootPreloadPose = mirrorIfRed(shootPreloadPoseBlue);
        pickUpMiddlePose = mirrorIfRed(pickUpMiddlePoseBlue);
        intakeMiddlePose = mirrorIfRed(intakeMiddlePoseBlue);
        openGatePose     = mirrorIfRed(openGatePoseBlue);
        shootMiddlePose  = mirrorIfRed(shootMiddlePoseBlue);
        pickUpTopPose    = mirrorIfRed(pickUpTopPoseBlue);
        intakeTopPose    = mirrorIfRed(intakeTopPoseBlue);
        shootTopPose     = mirrorIfRed(shootTopPoseBlue);
        pickUpBottomPose = mirrorIfRed(pickUpBottomPoseBlue);
        intakeBottomPose = mirrorIfRed(intakeBottomPoseBlue);
        shootBottomPose  = mirrorIfRed(shootBottomPoseBlue);
        parkPose         = mirrorIfRed(parkPoseBlue);

        // Control points
        cpPickUpMiddle1  = mirrorIfRed(cpPickUpMiddle1Blue);
        cpOpenGate1      = mirrorIfRed(cpOpenGate1Blue);
        cpOpenGate2      = mirrorIfRed(cpOpenGate2Blue);
        cpShootMiddle1   = mirrorIfRed(cpShootMiddle1Blue);
        cpShootTop1      = mirrorIfRed(cpShootTop1Blue);
        cpPickUpBottom1  = mirrorIfRed(cpPickUpBottom1Blue);
        cpShootBottom1   = mirrorIfRed(cpShootBottom1Blue);
        cpPark1          = mirrorIfRed(cpPark1Blue);
    }

    @Override
    protected Pose getStartPose() {
        // startPose is set in buildEffectivePoses(), called from buildPaths() in start()
        return startPose;
    }

    @Override
    protected void buildPaths() {
        // Build alliance-dependent poses first
        buildEffectivePoses();

        // Line 0: Shoot preload (startPose -> shootPreloadPose), straight line, Path
        shootPreloadPath = new Path(
                new BezierLine(startPose, shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                startPose.getHeading(),
                shootPreloadPose.getHeading(),
                0.65);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPreloadPose,
                        cpPickUpMiddle1,
                        pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        shootPreloadPose.getHeading(),
                        pickUpMiddlePose.getHeading(),
                        0.65)
                .setHeadingConstraint(0.975)
                .build();

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        intakeMiddlePath = new Path(
                new BezierLine(pickUpMiddlePose, intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(intakeMiddlePose.getHeading());

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        openGatePath = new Path(
                new BezierCurve(
                        intakeMiddlePose,
                        cpOpenGate1,
                        cpOpenGate2,
                        openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                intakeMiddlePose.getHeading(),
                openGatePose.getHeading(),
                0.8);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        shootMiddlePath = new Path(
                new BezierCurve(
                        openGatePose,
                        cpShootMiddle1,
                        shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                openGatePose.getHeading(),
                shootMiddlePose.getHeading(),
                0.65);
        shootMiddlePath.setHeadingConstraint(0.985);

        // Line 5: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootMiddlePose,
                        pickUpTopPose))
                .setLinearHeadingInterpolation(
                        shootMiddlePose.getHeading(),
                        pickUpTopPose.getHeading(),
                        0.65)
                .setHeadingConstraint(0.975)
                .build();

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        intakeTopPath = new Path(
                new BezierLine(pickUpTopPose, intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(intakeTopPose.getHeading());

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        shootTopPath = new Path(
                new BezierCurve(
                        intakeTopPose,
                        cpShootTop1,
                        shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                intakeTopPose.getHeading(),
                shootTopPose.getHeading(),
                0.65);

        // Line 8: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootTopPose,
                        cpPickUpBottom1,
                        pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        shootTopPose.getHeading(),
                        pickUpBottomPose.getHeading(),
                        0.65)
                .setHeadingConstraint(0.975)
                .build();

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        intakeBottomPath = new Path(
                new BezierLine(
                        pickUpBottomPose,
                        intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(intakeBottomPose.getHeading());

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        shootBottomPath = new Path(
                new BezierCurve(
                        intakeBottomPose,
                        cpShootBottom1,
                        shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                intakeBottomPose.getHeading(),
                shootBottomPose.getHeading(),
                0.65);

        // Line 11: Park (shootBottomPose -> parkPose), curve, Path
        parkPath = new Path(
                new BezierCurve(
                        shootBottomPose,
                        cpPark1,
                        parkPose));
        parkPath.setLinearHeadingInterpolation(
                shootBottomPose.getHeading(),
                parkPose.getHeading());
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to shoot preload
                follower.followPath(shootPreloadPath);
                setPathState(1001); // go to WAIT-before-preload-shoot state
                break;

            case 1001: // WAIT before shooting preload
                if (pathTimer.getElapsedTimeSeconds() > 1) {   // wait 0.75 sec
                    setPathState(100);                          // go to real shooting state
                }
                break;

            case 100:
                // Shoot 3 balls from far (preload)
                if (!shootingActive) {
                    startShootingBurst(3, Constants.LAUNCHER_FAR_VELOCITY);
                } else {
                    if (updateShootingBurst(Constants.LAUNCHER_FAR_VELOCITY)) {
                        setPathState(1);
                    }
                }
                break;

            case 1:
                // Pick up middle
                if (!follower.isBusy()) {
                    follower.followPath(pickUpMiddleChain, true); // hold at pickup pose
                    setPathState(2);
                }
                break;

            case 2:
                // Start intake + move to intake middle
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    startIntake();
                    follower.followPath(intakeMiddlePath);
                    setPathState(101); // intake middle cleanup
                }
                break;

            case 101:
                // Stop intake after reaching intake middle
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    setPathState(3);
                }
                break;

            case 3:
                // Open gate
                if (!follower.isBusy()) {
                    follower.followPath(openGatePath);
                    pathTimer.resetTimer();
                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {   // wait 0.75 sec
                    setPathState(4);                          // go to real shooting state
                }
                break;

            case 4:
                // Move to shoot middle pose
                if (!follower.isBusy()) {
                    follower.followPath(shootMiddlePath);
                    setPathState(1021); // WAIT-before-middle-shoot
                }
                break;

            case 1021: // WAIT before shooting middle
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {   // wait 0.75 sec
                    setPathState(102);                          // go to real shooting state
                }
                break;

            case 102:
                // Shoot 3 balls at middle
                if (!shootingActive) {
                    startShootingBurst(3, Constants.LAUNCHER_CLOSE_VELOCITY);
                } else {
                    if (updateShootingBurst(Constants.LAUNCHER_CLOSE_VELOCITY)) {
                        setPathState(5);
                    }
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
                // Start intake + move to intake top
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    startIntake();
                    follower.followPath(intakeTopPath);
                    setPathState(103); // intake top cleanup
                }
                break;

            case 103:
                // Stop intake after reaching intake top
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    setPathState(7);
                }
                break;

            case 7:
                // Move to shoot top pose
                if (!follower.isBusy()) {
                    follower.followPath(shootTopPath);
                    setPathState(104); // shooting burst at top (no extra wait here yet)
                }
                break;

            case 104:
                // Shoot 3 balls at top
                if (!shootingActive) {
                    startShootingBurst(3, Constants.LAUNCHER_CLOSE_VELOCITY);
                } else {
                    if (updateShootingBurst(Constants.LAUNCHER_CLOSE_VELOCITY)) {
                        setPathState(8);
                    }
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
                // Start intake + move to intake bottom
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    startIntake();
                    follower.followPath(intakeBottomPath);
                    setPathState(105); // intake bottom cleanup
                }
                break;

            case 105:
                // Stop intake after reaching intake bottom
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    stopIntake();
                    setPathState(10);
                }
                break;

            case 10:
                // Move to shoot bottom pose
                if (!follower.isBusy()) {
                    follower.followPath(shootBottomPath);
                    setPathState(106); // shooting burst at bottom (no extra wait yet)
                }
                break;

            case 106:
                // Shoot 3 balls at bottom
                if (!shootingActive) {
                    startShootingBurst(3, Constants.LAUNCHER_FAR_VELOCITY);
                } else {
                    if (updateShootingBurst(Constants.LAUNCHER_FAR_VELOCITY)) {
                        setPathState(11);
                    }
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
