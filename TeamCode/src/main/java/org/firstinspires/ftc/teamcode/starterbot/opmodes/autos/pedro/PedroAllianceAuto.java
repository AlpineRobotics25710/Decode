// PedroAllianceAuto.java
package org.firstinspires.ftc.teamcode.starterbot.opmodes.autos.pedro;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.enums.Alliance;

public abstract class PedroAllianceAuto extends PedroBaseAuto {
    protected Alliance alliance = Alliance.BLUE;

    /**
     * Helper:
     * BLUE -> return pose as-is
     * RED  -> mirror pose using PedroPathing 2.0.4
     */
    protected Pose alliancePose(Pose bluePose) {
        return (alliance == Alliance.BLUE) ? bluePose : bluePose.mirror();
    }

    @Override
    public void init() {
        super.init();
        CommonTelemetry.addData("Autonomous", "Alliance Select");
        CommonTelemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.yWasPressed()) {
            alliance = Alliance.RED;
        }

        CommonTelemetry.addData("Alliance Select", "X/Square = BLUE, Y/Triangle = RED");
        CommonTelemetry.addData("Alliance", alliance);
        CommonTelemetry.update();
    }
}
