package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro.poses.FarPoses;

@Autonomous(name = "Far Side Preload Park", group = "pedro")
public class PedroPreload extends PedroBaseAuto {
    private FarPoses poses;

    @Override
    protected Pose getStartPose() {
        return poses.startPose;
    }

    @Override
    protected void allianceSetup(Alliance alliance) {
        poses = new FarPoses();
        if (alliance != poses.originalPosesAlliance()) {
            poses.mirror();
        }
    }

    @Override
    protected Pose getEndPose() {
        return poses.preloadParkPose;
    }

    @Override
    protected void buildPaths() {
        // Line 0: Shoot preload (startPose -> shootFarPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        PathChain shootPreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.startPose,
                        poses.shootFarPose))
                .setLinearHeadingInterpolation(
                        poses.startPose.getHeading(),
                        poses.shootFarPose.getHeading())
                .build();
        addPath(shootPreloadPath);
        addShot(shootPreloadPath);

        // Line 0: Shoot preload (startPose -> shootFarPose), straight line, Path
        // Only pick-up paths are PathChains, everything else is a Path
        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(
                        poses.shootFarPose,
                        poses.preloadParkPose))
                .setLinearHeadingInterpolation(
                        poses.shootFarPose.getHeading(),
                        poses.preloadParkPose.getHeading())
                .build();
        addPath(park);
    }
}
