package org.firstinspires.ftc.teamcode.starterbot.opmode.teleop;

import android.annotation.SuppressLint;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

@TeleOp(group = "testers")
public class TfodTest extends BaseTeleOp {
    private VisionPortal visionPortal;
    private TfodProcessor tfodProcessor;
    public static String appContext = "org.firstinspires.ftc.teamcode";

    @Override
    public void initGamepads() {
        driver = gamepad1;
        operator = gamepad1;

        turtleMode = () -> gamepad1.right_trigger > 0.05;
    }

    @Override
    public void init() {
        //super.init();
        CommonTelemetry.init(telemetry);

        try {
            AssetManager am = hardwareMap.appContext.getAssets();
            String[] files = am.list("");
            CommonTelemetry.debug("Assets: " + Arrays.toString(files));
        } catch (IOException e) {
            CommonTelemetry.debug("Error listing assets: " + e.getMessage());
        }

        try {
            // Load model and labels from assets
            Interpreter tflite = new Interpreter(FileUtil.loadMappedFile(hardwareMap.appContext, "model.tflite"));
            List<String> labels = FileUtil.loadLabels(hardwareMap.appContext, "labels.txt");

            CommonTelemetry.debug(hardwareMap.appContext);

            // Create custom TensorFlow processor
            tfodProcessor = new TfodProcessor(tflite, labels, 224);

            // Build VisionPortal and attach our processor
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(tfodProcessor)
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .setAutoStopLiveView(false) // keep the stream active for onDrawFrame()
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .build();

            CommonTelemetry.debug("TF Lite model loaded successfully.");
        } catch (IOException e) {
            CommonTelemetry.debug("Failed to load model or labels: " + e.getMessage());
        }

        CommonTelemetry.update();
    }

    @Override
    public void init_loop() {
        if (visionPortal == null) {
            try {
                Interpreter tflite = new Interpreter(FileUtil.loadMappedFile(hardwareMap.appContext, "model.tflite"));
                List<String> labels = FileUtil.loadLabels(hardwareMap.appContext, "labels.txt");

                tfodProcessor = new TfodProcessor(tflite, labels, 224);

                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(tfodProcessor)
                        .enableLiveView(true)
                        .setCameraResolution(new Size(640, 480))
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setAutoStopLiveView(false)
                        .build();

                CommonTelemetry.debug("VisionPortal started in init_loop.");
            } catch (Exception e) {
                CommonTelemetry.debug("Error: " + e.getMessage());
            }
        }

        CommonTelemetry.addData("app context", hardwareMap.appContext);
        CommonTelemetry.update();
    }

    @Override
    public void loop() {
        CommonTelemetry.debug("Running TF Lite Vision...");
        CommonTelemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    /**
     * Custom TensorFlow Lite processor that draws detections on the camera stream.
     */
    public static class TfodProcessor implements VisionProcessor {
        private final Interpreter tflite;
        private final List<String> labels;
        private final int inputSize;
        private final Paint paint = new Paint();
        private final float CONFIDENCE_THRESHOLD = 0.5f;

        private volatile float[][] lastOutput;  // store most recent inference result

        public TfodProcessor(Interpreter tflite, List<String> labels, int inputSize) {
            this.tflite = tflite;
            this.labels = labels;
            this.inputSize = inputSize;

            paint.setColor(Color.RED);
            paint.setStrokeWidth(6);
            paint.setTextSize(60);
            paint.setStyle(Paint.Style.FILL_AND_STROKE);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Called once when camera starts streaming
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert Mat -> Bitmap
            Bitmap bmp = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bmp);

            // Resize to model input
            Bitmap resized = Bitmap.createScaledBitmap(bmp, inputSize, inputSize, true);

            // Convert to normalized float32 tensor
            float[][][][] input = new float[1][inputSize][inputSize][3];
            for (int y = 0; y < inputSize; y++) {
                for (int x = 0; x < inputSize; x++) {
                    int pixel = resized.getPixel(x, y);
                    input[0][y][x][0] = ((pixel >> 16) & 0xFF) / 255.0f;
                    input[0][y][x][1] = ((pixel >> 8) & 0xFF) / 255.0f;
                    input[0][y][x][2] = (pixel & 0xFF) / 255.0f;
                }
            }

            // Run inference
            float[][] output = new float[1][labels.size()];
            tflite.run(input, output);

            lastOutput = output;
            return output;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            float[][] output = (float[][]) userContext;
            if (output == null) output = lastOutput; // fallback to last known result

            if (output != null) {
                int topIdx = getTopPrediction(output[0]);
                float confidence = output[0][topIdx];

                if (confidence > CONFIDENCE_THRESHOLD) {
                    String label = (labels != null && topIdx < labels.size())
                            ? labels.get(topIdx)
                            : "Class " + topIdx;

                    @SuppressLint("DefaultLocale") String text = String.format("%s (%.2f)", label, confidence);
                    // Draw centered on the preview
                    canvas.drawText(text, 80, 120, paint);
                }
            }
        }

        private int getTopPrediction(float[] probs) {
            int bestIdx = 0;
            for (int i = 1; i < probs.length; i++) {
                if (probs[i] > probs[bestIdx]) bestIdx = i;
            }
            return bestIdx;
        }
    }
}
