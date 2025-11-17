package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Far Side Red", group = "Pedro Autos")
public class FarSideRed extends PedroBaseAuto {

    // POSES (RED)

    // Global start pose
    private final Pose startPose          = new Pose(87.5, 8.75, Math.toRadians(90));

    private final Pose shootPreloadPose   = new Pose(90,   15.5, FIRST_FAR_SHOOTING_ANGLE_RED);   // line 0: Shoot preload
    private final Pose pickUpMiddlePose   = new Pose(94,   58.5, Math.toRadians(0));    // line 1: Pick up middle
    private final Pose intakeMiddlePose   = new Pose(126.5,58.5, Math.toRadians(0));  // line 2: Intake middle
    private final Pose openGatePose       = new Pose(129,  70.5, Math.toRadians(180));  // line 3: Open gate
    private final Pose shootMiddlePose    = new Pose(88,   88,   CLOSE_SHOOTING_ANGLE_RED);   // line 4: Shoot middle
    private final Pose pickUpTopPose      = new Pose(94,   83,   Math.toRadians(0));    // line 5: Pick up top
    private final Pose intakeTopPose      = new Pose(128,  83,   Math.toRadians(0));  // line 6: Intake top
    private final Pose shootTopPose       = new Pose(88,   88,   CLOSE_SHOOTING_ANGLE_RED);   // line 7: Shoot top
    private final Pose pickUpBottomPose   = new Pose(94,   35,   Math.toRadians(0));    // line 8: Pick up bottom
    private final Pose intakeBottomPose   = new Pose(128,  35,   Math.toRadians(0));  // line 9: Intake bottom
    private final Pose shootBottomPose    = new Pose(90,   15.5, LAST_FAR_SHOOTING_ANGLE_RED);   // line 10: Shoot bottom
    private final Pose parkPose           = new Pose(116,  70, Math.toRadians(180));  // line 11: Park

    // Control points from RedSideFarAuto-Decode.pp
    private final Pose cpPickUpMiddle1  = new Pose(90.75, 56,   0);   // line 1
    private final Pose cpOpenGate1      = new Pose(93,    58,   0);   // line 3
    private final Pose cpOpenGate2      = new Pose(85.5,  64,   0);   // line 3
    private final Pose cpShootMiddle1   = new Pose(85,    57.5,0);   // line 4
    private final Pose cpShootTop1      = new Pose(95,    92,   0);   // line 7
    private final Pose cpPickUpBottom1  = new Pose(86,    35.5,   0);   // line 8
    private final Pose cpShootBottom1   = new Pose(107.25,17.5, 0);   // line 10
    private final Pose cpPark1          = new Pose(98,    60,   0);   // line 11

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
                startPose.getHeading(),           // from startDeg of line 0 / startPose
                shootPreloadPose.getHeading(),           // endDeg of line 0
                0.65);

        // Line 1: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPreloadPose,
                        cpPickUpMiddle1,
                        pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        shootPreloadPose.getHeading(),     // startDeg line 1
                        pickUpMiddlePose.getHeading(),      // endDeg line 1
                        0.65)
                .setHeadingConstraint(0.98)
                .build();

        // Line 2: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        intakeMiddlePath = new Path(
                new BezierLine(pickUpMiddlePose, intakeMiddlePose));
        // headingType = constant, startDeg = endDeg = 180
        intakeMiddlePath.setConstantHeadingInterpolation(0);

        // Line 3: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        openGatePath = new Path(
                new BezierCurve(
                        intakeMiddlePose,
                        cpOpenGate1,
                        cpOpenGate2,
                        openGatePose));
        // headingType = linear, startDeg = 0, endDeg = 180
        openGatePath.setLinearHeadingInterpolation(
                pickUpMiddlePose.getHeading(),
                openGatePose.getHeading(),
                0.8);

        // Line 4: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        shootMiddlePath = new Path(
                new BezierCurve(
                        openGatePose,
                        cpShootMiddle1,
                        shootMiddlePose));
        // headingType = linear, startDeg = 180, endDeg = 45
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
                // headingType = linear, startDeg = 45, endDeg = 0
                .setLinearHeadingInterpolation(
                        shootMiddlePose.getHeading(),
                        pickUpTopPose.getHeading(),
                        0.65)
                .setHeadingConstraint(0.98)
                .build();

        // Line 6: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        intakeTopPath = new Path(
                new BezierLine(pickUpTopPose, intakeTopPose));
        // headingType = constant, 180 deg
        intakeTopPath.setConstantHeadingInterpolation(intakeTopPose.getHeading());

        // Line 7: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        shootTopPath = new Path(
                new BezierCurve(
                        intakeTopPose,
                        cpShootTop1,
                        shootTopPose));
        // headingType = linear, startDeg = 0, endDeg = 45
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
                // headingType = linear, startDeg = 45, endDeg = 0
                .setLinearHeadingInterpolation(
                        shootTopPose.getHeading(),
                        pickUpBottomPose.getHeading(),
                        0.65)
                .setHeadingConstraint(0.98)
                .build();

        // Line 9: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        intakeBottomPath = new Path(
                new BezierLine(
                        pickUpBottomPose,
                        intakeBottomPose));
        // headingType = constant, 180 deg
        intakeBottomPath.setConstantHeadingInterpolation(intakeBottomPose.getHeading());

        // Line 10: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        shootBottomPath = new Path(
                new BezierCurve(
                        intakeBottomPose,
                        cpShootBottom1,
                        shootBottomPose));
        // headingType = linear, startDeg = 180, endDeg = 66
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
        // headingType = linear, startDeg = 66, endDeg = 180
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
                pathTimer.resetTimer();
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
                if (pathTimer.getElapsedTimeSeconds() > 1) {   // wait 1 sec
                    setPathState(4);                          // go to real shooting state
                }
                break;

            case 4:
                // Move to shoot middle pose
                if (!follower.isBusy()) {
                    follower.followPath(shootMiddlePath);
                    pathTimer.resetTimer();
                    setPathState(1021); // WAIT-before-middle-shoot
                }
                break;

            case 1021: // WAIT before shooting middle
                if (pathTimer.getElapsedTimeSeconds() > 1) {   // wait 1 sec
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
                    setPathState(104); // shooting burst at top
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
                    setPathState(106); // shooting burst at bottom
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
