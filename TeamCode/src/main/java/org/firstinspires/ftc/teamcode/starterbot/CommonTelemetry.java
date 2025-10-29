package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CommonTelemetry {
    private static TelemetryManager panelsTelemetry;
    private static Telemetry robotTelemetry;

    // Make singleton
    private CommonTelemetry() {
    }

    public static void init(Telemetry robotTelemetry) {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        CommonTelemetry.robotTelemetry = robotTelemetry;
    }

    public static void debug(Object... data) {
        try {
            panelsTelemetry.debug(data);
        } catch (NullPointerException e) {
            throw new NullPointerException("Did you forget to call init and initialize common telemetry in your OpMode?");
        }
    }

    public static void addData(String key, Object value) {
        try {
            panelsTelemetry.debug(key + ": " + value);
        } catch (NullPointerException e) {
            throw new NullPointerException("Did you forget to call init and initialize common telemetry in your OpMode?");
        }
    }

    public static void update() {
        try {
            panelsTelemetry.update(robotTelemetry);
        } catch (NullPointerException e) {
            throw new NullPointerException("Did you forget to call init and initialize common telemetry in your OpMode?");
        }
    }
}
