package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.Constants;

@Autonomous(name = "Blue Side Close Auto", group = "Pedro Autos")
public class BlueSideCloseAuto extends PedroBaseAuto {

    // poses

    // Heading 323.5° to match first real segment
    private final Pose startPose          = new Pose(21,   122,  Math.toRadians(323.5));

    private final Pose shootPreloadPose   = new Pose(60,   83.5, CLOSE_SHOOTING_ANGLE_BLUE); // line 1
    private final Pose pickUpMiddlePose   = new Pose(45,   60,   Math.toRadians(180)); // line 2
    private final Pose intakeMiddlePose   = new Pose(17.5, 60,   Math.toRadians(180)); // line 3
    private final Pose openGatePose       = new Pose(15, 70.5,   Math.toRadians(0));   // line 4
    private final Pose shootMiddlePose    = new Pose(60,   84, CLOSE_SHOOTING_ANGLE_BLUE); // line 5
    private final Pose pickUpTopPose      = new Pose(45,   84,   Math.toRadians(180)); // line 6
    private final Pose intakeTopPose      = new Pose(17.5, 84,   Math.toRadians(180)); // line 7
    private final Pose shootTopPose       = new Pose(60,   84, CLOSE_SHOOTING_ANGLE_BLUE); // line 8
    private final Pose pickUpBottomPose   = new Pose(45,   36,   Math.toRadians(180)); // line 9
    private final Pose intakeBottomPose   = new Pose(22,   36,   Math.toRadians(180)); // line 10
    private final Pose shootBottomPose    = new Pose(60,   84, CLOSE_SHOOTING_ANGLE_BLUE); // line 11
    private final Pose parkPose           = new Pose(20,   70,   Math.toRadians(0));   // line 12

    // Control points
    private final Pose cpShootPreload1   = new Pose(62.75, 107.75, 0); // line 1
    private final Pose cpPickUpMiddle1   = new Pose(64,    63,     0); // line 2
    private final Pose cpOpenGate1       = new Pose(51,    58,     0); // line 4
    private final Pose cpOpenGate2       = new Pose(59.25, 70,     0); // line 4
    private final Pose cpShootMiddle1    = new Pose(60,    67.25,  0); // line 5
    private final Pose cpShootTop1       = new Pose(53,    68.25,  0); // line 8
    private final Pose cpPickUpBottom1   = new Pose(60,    34,     0); // line 9
    private final Pose cpShootBottom1    = new Pose(58,    56,     0); // line 11
    private final Pose cpPark1           = new Pose(54.5,  69,     0); // line 12

    // Paths/PathChains
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
        // Line 1: Shoot preload (startPose -> shootPreloadPose), curve, Path
        shootPreloadPath = new Path(
                new BezierCurve(
                        startPose,
                        cpShootPreload1,
                        shootPreloadPose));
        shootPreloadPath.setLinearHeadingInterpolation(
                Math.toRadians(323.5),
                Math.toRadians(131));

        // Line 2: Pick up middle (shootPreloadPose -> pickUpMiddlePose), curve, PathChain
        pickUpMiddleChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPreloadPose,
                        cpPickUpMiddle1,
                        pickUpMiddlePose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(131),
                        Math.toRadians(180))
                .setHeadingConstraint(0.975)
                .build();

        // Line 3: Intake middle (pickUpMiddlePose -> intakeMiddlePose), straight, Path
        intakeMiddlePath = new Path(
                new BezierLine(
                        pickUpMiddlePose,
                        intakeMiddlePose));
        intakeMiddlePath.setConstantHeadingInterpolation(Math.toRadians(180));

        // Line 4: Open gate (intakeMiddlePose -> openGatePose), curve, Path
        openGatePath = new Path(
                new BezierCurve(
                        intakeMiddlePose,
                        cpOpenGate1,
                        cpOpenGate2,
                        openGatePose));
        openGatePath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(0));

        // Line 5: Shoot middle (openGatePose -> shootMiddlePose), curve, Path
        shootMiddlePath = new Path(
                new BezierCurve(
                        openGatePose,
                        cpShootMiddle1,
                        shootMiddlePose));
        shootMiddlePath.setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(131));

        // Line 6: Pick up top (shootMiddlePose -> pickUpTopPose), straight, PathChain
        pickUpTopChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootMiddlePose,
                        pickUpTopPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(131),
                        Math.toRadians(180))
                .setHeadingConstraint(0.975)
                .build();

        // Line 7: Intake top (pickUpTopPose -> intakeTopPose), straight, Path
        intakeTopPath = new Path(
                new BezierLine(
                        pickUpTopPose,
                        intakeTopPose));
        intakeTopPath.setConstantHeadingInterpolation(Math.toRadians(180));

        // Line 8: Shoot top (intakeTopPose -> shootTopPose), curve, Path
        shootTopPath = new Path(
                new BezierCurve(
                        intakeTopPose,
                        cpShootTop1,
                        shootTopPose));
        shootTopPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(131));

        // Line 9: Pick up bottom (shootTopPose -> pickUpBottomPose), curve, PathChain
        pickUpBottomChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootTopPose,
                        cpPickUpBottom1,
                        pickUpBottomPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(131),
                        Math.toRadians(180))
                .setHeadingConstraint(0.975)
                .build();

        // Line 10: Intake bottom (pickUpBottomPose -> intakeBottomPose), straight, Path
        intakeBottomPath = new Path(
                new BezierLine(
                        pickUpBottomPose,
                        intakeBottomPose));
        intakeBottomPath.setConstantHeadingInterpolation(Math.toRadians(180));

        // Line 11: Shoot bottom (intakeBottomPose -> shootBottomPose), curve, Path
        shootBottomPath = new Path(
                new BezierCurve(
                        intakeBottomPose,
                        cpShootBottom1,
                        shootBottomPose));
        shootBottomPath.setLinearHeadingInterpolation(
                Math.toRadians(180),
                Math.toRadians(131));

        // Line 12: Park (shootBottomPose -> parkPose), curve, Path
        parkPath = new Path(
                new BezierCurve(
                        shootBottomPose,
                        cpPark1,
                        parkPose));
        parkPath.setLinearHeadingInterpolation(
                Math.toRadians(131),
                Math.toRadians(0));
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to shoot preload
                follower.followPath(shootPreloadPath);
                setPathState(100);
                break;

            case 100:
                // Shoot 3 balls from far
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
                    follower.followPath(pickUpMiddleChain, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Start intake + move to intake middle
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    startIntake();
                    follower.followPath(intakeMiddlePath);
                    setPathState(101); // intake middle cleanup
                }
                break;

            case 101:
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
                    setPathState(4);
                }
                break;

            case 4:
                // Move to shoot middle pose
                if (!follower.isBusy()) {
                    follower.followPath(shootMiddlePath);
                    setPathState(102); // shooting burst at middle
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
                    follower.setMaxPower(0.6);
                    startIntake();
                    follower.followPath(intakeTopPath);
                    setPathState(103); // intake top cleanup
                }
                break;

            case 103:
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
                    follower.setMaxPower(0.6);
                    startIntake();
                    follower.followPath(intakeBottomPath);
                    setPathState(105); // intake bottom cleanup
                }
                break;

            case 105:
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
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }
}
