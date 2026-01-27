package org.firstinspires.ftc.teamcode.starterbot.opmodes.teleops.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(group = "testers")
public class VoltageLogger extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup for CSV logging
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
        File logFile = new File(AppUtil.getDefContext().getFilesDir(), "voltage_log_" + timestamp + ".csv");
        PrintWriter writer = null;

        try {
            writer = new PrintWriter(new FileWriter(logFile, true));
            // Write CSV header
            writer.println("Timestamp,Device,Voltage");
        } catch (IOException e) {
            telemetry.addData("Error", "Could not open log file: " + e.getMessage());
            telemetry.update();
            sleep(2000); // give user time to see error
            requestOpModeStop(); // stop opmode if file can't be opened
            return;
        }


        waitForStart();

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis() - startTime;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                String deviceName = hardwareMap.getNamesOf(sensor).iterator().next();
                double voltage = sensor.getVoltage();
                telemetry.addData(deviceName, voltage);

                // Log to CSV
                writer.println(String.format(Locale.getDefault(), "%d,%s,%.2f", currentTime, deviceName, voltage));
            }
            telemetry.update();
            // Optional: add a small delay to control logging frequency
            sleep(100);
        }

        // Close the writer
        writer.close();
    }
}
