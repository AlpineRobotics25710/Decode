package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import static org.firstinspires.ftc.teamcode.starterbot.Constants.LAUNCHER_CLOSE_VELOCITY;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.starterbot.Robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class FarSideBlue extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int currentPathIndex = 0; // Current autonomous path index
    private List<PathChain> paths;
    private Set<Integer> requiresShot;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new ArrayList<>();
        requiresShot = new HashSet<>();
        generatePaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        currentPathIndex = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Current path index", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        if (currentPathIndex >= paths.size()) {
            return currentPathIndex; // All paths done
        }

        // If not busy, start next path
        if (!follower.isBusy()) {
            if (requiresShot.contains(currentPathIndex)) {
                Robot.launchTimeDelay(LAUNCHER_CLOSE_VELOCITY);
            }

            follower.followPath(paths.get(currentPathIndex));
            currentPathIndex++;
        }

        return currentPathIndex;
    }
    public void generatePaths() {
        paths.add(follower.pathBuilder() // Setup empty 1
                .addPath(
                        new BezierCurve(
                                new Pose(57.000, 9.300),
                                new Pose(68.108, 51.302),
                                new Pose(48.000, 70.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build()
        );

        paths.add(follower.pathBuilder() // Empty 1
                .addPath(
                        new BezierLine(new Pose(48.000, 70.000), new Pose(16.500, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build()
        );

        paths.add(follower.pathBuilder() // Setup pickup 1
                .addPath(
                        new BezierLine(new Pose(16.500, 70.000), new Pose(48.000, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build()
        );

        paths.add(follower // Setup pickup 1
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48.000, 70.000), new Pose(48.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build());

        paths.add(follower // Pickup 1
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48.000, 84.000), new Pose(16.500, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build());

        paths.add(follower // Shoot 1
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.500, 84.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build());
        requiresShot.add(5);

        paths.add(follower // setup pickup 2
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(48.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build());

        paths.add(follower // pickup 2
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48.000, 60.000), new Pose(16.500, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build());

        paths.add(follower // shoot 2
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.500, 60.000),
                                new Pose(61.032, 58.909),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build());
        requiresShot.add(9);

        paths.add(follower // setup pickup 3
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(65.101, 54.840),
                                new Pose(48.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build());

        paths.add(follower // pickup 3
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48.000, 36.000), new Pose(16.500, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build());

        paths.add(follower // shoot 3
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.500, 36.000),
                                new Pose(58.025, 42.457),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build());
        requiresShot.add(12);

        paths.add(follower // SetupEmpty2
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(48.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build());

        paths.add(follower // Empty2
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48.000, 70.000), new Pose(16.500, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build());

        paths.add(follower // Leave
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.500, 70.000), new Pose(36.000, 70.000))
                )
                .setTangentHeadingInterpolation()
                .build());
    }
}
