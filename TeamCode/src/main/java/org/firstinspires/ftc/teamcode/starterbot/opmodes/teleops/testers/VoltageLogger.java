package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.teamcode.starterbot.Robot;
import org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.prod.OneDriverTeleOp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp(group = "testers")
public class VoltageLogger extends OneDriverTeleOp {

    private final List<String> voltageSensorNames = new ArrayList<>();
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private PrintWriter writer = null;
    private long startTime;

    // ---- LOG RATE LIMITING (CRITICAL FIX) ----
    private static final long LOG_INTERVAL_MS = 100; // 10 Hz
    private long lastLogTime = 0;

    @Override
    public void init() {
        super.init();

        // --- Discover Voltage Sensors ---
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            hardwareMap.getNamesOf(sensor)
                    .stream()
                    .findFirst()
                    .ifPresent(name -> {
                        voltageSensorNames.add(name + "_Voltage");
                        voltageSensors.add(sensor);
                    });
        }

        try {
            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss", Locale.US).format(new Date());

            File logFile = new File(
                    AppUtil.getDefContext().getFilesDir(),
                    "voltage_log_" + timestamp + ".csv"
            );

            writer = new PrintWriter(new FileWriter(logFile, true));

            // ---- CSV HEADER ----
            String header =
                    "Timestamp," +
                            String.join(",", voltageSensorNames) + "," +
                            "frontLeftDrive_Power,frontRightDrive_Power," +
                            "backLeftDrive_Power,backRightDrive_Power," +
                            "launcher_Power,leftIntake_Power,rightIntake_Power," +
                            "leftFeeder_Power,rightFeeder_Power," +
                            "ramp_Position,ramp2_Position,blocker_Position";

            writer.println(header);
            writer.flush();

        } catch (IOException e) {
            telemetry.addData("Logger Error", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {
        CommonTelemetry.addData("file path", AppUtil.getDefContext().getFilesDir() + "/voltage_log_" + ".csv");
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        startTime = System.currentTimeMillis();
        lastLogTime = 0;
    }

    @Override
    public void loop() {
        // ---- DRIVE LOGIC (MUST RUN FAST) ----
        super.loop();

        if (writer == null) return;

        long now = System.currentTimeMillis();

        // ---- RATE LIMIT FILE I/O ----
        if (now - lastLogTime < LOG_INTERVAL_MS) return;
        lastLogTime = now;

        long elapsed = now - startTime;
        StringBuilder logLine = new StringBuilder();

        logLine.append(elapsed).append(",");

        // ---- Voltages ----
        for (VoltageSensor sensor : voltageSensors) {
            logLine.append(
                    String.format(Locale.US, "%.2f", sensor.getVoltage())
            ).append(",");
        }

        // ---- Motor Powers ----
        logLine.append(Robot.frontLeftDrive.getPower()).append(",");
        logLine.append(Robot.frontRightDrive.getPower()).append(",");
        logLine.append(Robot.backLeftDrive.getPower()).append(",");
        logLine.append(Robot.backRightDrive.getPower()).append(",");
        logLine.append(Robot.launcher.getPower()).append(",");
        logLine.append(Robot.leftIntake.getPower()).append(",");
        logLine.append(Robot.rightIntake.getPower()).append(",");

        // ---- CRServo Powers ----
        logLine.append(Robot.leftFeeder.getPower()).append(",");
        logLine.append(Robot.rightFeeder.getPower()).append(",");

        // ---- Servo Positions ----
        logLine.append(Robot.ramp.getPosition()).append(",");
        logLine.append(Robot.ramp2.getPosition()).append(",");
        logLine.append(Robot.blocker.getPosition());

        writer.println(logLine);
    }

    @Override
    public void stop() {
        super.stop();

        if (writer != null) {
            writer.flush();
            writer.close();
        }

        Robot.stopAll();
    }
}
