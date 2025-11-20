package org.firstinspires.ftc.teamcode.starterbot.opmode.teleop;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.starterbot.CommonTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "testers")
public class TfodTest extends BaseTeleOp {

    private VisionPortal visionPortal;
    private TfodProcessor tfodProcessor;

    private static final int MODEL_INPUT_SIZE = 224;
    private static final float CONFIDENCE_THRESHOLD = 0.3f;
    private static final float IOU_THRESHOLD = 0.5f;

    @Override
    public void initGamepads() {
        driver = gamepad1;
        operator = gamepad1;

        turtleMode = () -> gamepad1.right_trigger > 0.05;
    }

    @Override
    public void init() {
        CommonTelemetry.init(telemetry);

        try {
            // Load model and labels from assets
            Interpreter tflite = new Interpreter(FileUtil.loadMappedFile(hardwareMap.appContext, "model.tflite"));
            List<String> labels = FileUtil.loadLabels(hardwareMap.appContext, "labels.txt");

            tfodProcessor = new TfodProcessor(tflite, labels, MODEL_INPUT_SIZE, CONFIDENCE_THRESHOLD, IOU_THRESHOLD);

            // Build VisionPortal
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(tfodProcessor)
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .setAutoStopLiveView(false)
                    .build();

            CommonTelemetry.debug("TF Lite model loaded successfully.");
        } catch (IOException e) {
            CommonTelemetry.debug("Failed to load model or labels: " + e.getMessage());
        }

        List<TfodProcessor.Detection> detections = tfodProcessor.lastDetections;

        if (detections == null || detections.isEmpty()) {
            CommonTelemetry.addData("Detections", 0);
        } else {
            CommonTelemetry.addData("Detections", detections.size());

            // Print each detection
            for (int i = 0; i < detections.size(); i++) {
                TfodProcessor.Detection det = detections.get(i);
                CommonTelemetry.addData("Detection " + i,
                        det.className + String.format(" (%.2f)", det.confidence));
            }
        }

        CommonTelemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        if (tfodProcessor != null) {
            // Get the latest detections
            List<TfodProcessor.Detection> detections = tfodProcessor.lastDetections;

            if (detections == null || detections.isEmpty()) {
                CommonTelemetry.addData("Detections", 0);
            } else {
                CommonTelemetry.addData("Detections", detections.size());

                // Print each detection
                for (int i = 0; i < detections.size(); i++) {
                    TfodProcessor.Detection det = detections.get(i);
                    CommonTelemetry.addData("Detection " + i,
                            det.className + String.format(" (%.2f)", det.confidence));
                }
            }
        }

        CommonTelemetry.update();
    }

    @Override
    public void loop() {
        CommonTelemetry.debug("Running YOLO TF Lite Vision...");
        CommonTelemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    /**
     * YOLO TF Lite processor for VisionPortal
     */
    public static class TfodProcessor implements VisionProcessor {

        private final Interpreter tflite;
        private final List<String> labels;
        private final int inputSize;
        private final float confidenceThreshold;
        private final float iouThreshold;

        private final Paint boxPaint = new Paint();
        private final Paint textPaint = new Paint();

        private volatile List<Detection> lastDetections = new ArrayList<>();

        public TfodProcessor(Interpreter tflite, List<String> labels, int inputSize,
                             float confidenceThreshold, float iouThreshold) {
            this.tflite = tflite;
            this.labels = labels;
            this.inputSize = inputSize;
            this.confidenceThreshold = confidenceThreshold;
            this.iouThreshold = iouThreshold;

            boxPaint.setColor(Color.RED);
            boxPaint.setStrokeWidth(6);
            boxPaint.setStyle(Paint.Style.STROKE);

            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(50);
            textPaint.setStyle(Paint.Style.FILL);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Not needed
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert Mat -> normalized input tensor
            float[][][][] input = matToInputTensor(frame, inputSize);

            // YOLO output: [1, num_boxes, box_data]
            int[] shape = tflite.getOutputTensor(0).shape();
            float[][][] output = new float[shape[0]][shape[1]][shape[2]];

            tflite.run(input, output);

            // Parse output to Detection objects
            List<Detection> detections = parseYoloOutput(output[0], labels, confidenceThreshold, iouThreshold);
            lastDetections = detections;
            return detections;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            List<Detection> detections = userContext != null ? (List<Detection>) userContext : lastDetections;
            if (detections == null) return;

            for (Detection det : detections) {
                float x1 = det.x1 * onscreenWidth;
                float y1 = det.y1 * onscreenHeight;
                float x2 = det.x2 * onscreenWidth;
                float y2 = det.y2 * onscreenHeight;

                // Draw bounding box
                canvas.drawRect(x1, y1, x2, y2, boxPaint);

                // Draw label
                @SuppressLint("DefaultLocale") String text = det.className + String.format(" %.2f", det.confidence);
                canvas.drawText(text, x1, Math.max(0, y1 - 10), textPaint);
            }
        }

        // Convert Mat -> normalized float input
        private float[][][][] matToInputTensor(Mat frame, int inputSize) {
            Bitmap bmp = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bmp);
            //Bitmap resized = Bitmap.createScaledBitmap(bmp, inputSize, inputSize, true);

            Matrix matrix = new Matrix();
            matrix.postRotate(90); // or 270, depending on orientation
            Bitmap resized = Bitmap.createScaledBitmap(bmp, inputSize, inputSize, true);

            float[][][][] input = new float[1][inputSize][inputSize][3];
            for (int y = 0; y < inputSize; y++) {
                for (int x = 0; x < inputSize; x++) {
                    int pixel = resized.getPixel(x, y);
                    input[0][y][x][0] = ((pixel >> 16) & 0xFF) / 255f;
                    input[0][y][x][1] = ((pixel >> 8) & 0xFF) / 255f;
                    input[0][y][x][2] = (pixel & 0xFF) / 255f;
                }
            }
            return input;
        }

        // Parse YOLO output tensor to detections
        private List<Detection> parseYoloOutput(float[][] output, List<String> labels,
                                                float confThreshold, float iouThreshold) {
            List<Detection> detections = new ArrayList<>();

            int numBoxes = output.length;
            int numClasses = output[0].length - 5;

            for (int i = 0; i < numBoxes; i++) {
                float conf = output[i][4];
                if (conf < confThreshold) continue;

                // Get class with max probability
                int bestClass = 0;
                float bestProb = 0f;
                for (int c = 0; c < numClasses; c++) {
                    if (output[i][5 + c] > bestProb) {
                        bestProb = output[i][5 + c];
                        bestClass = c;
                    }
                }

                float finalConf = conf * bestProb;
                if (finalConf < confThreshold) continue;

                // Box coordinates (normalized 0-1)
                float cx = output[i][0];
                float cy = output[i][1];
                float w = output[i][2];
                float h = output[i][3];
                float x1 = cx - w / 2f;
                float y1 = cy - h / 2f;
                float x2 = cx + w / 2f;
                float y2 = cy + h / 2f;

                String label = labels != null && bestClass < labels.size() ? labels.get(bestClass) : "Class" + bestClass;
                detections.add(new Detection(x1, y1, x2, y2, label, finalConf));
            }

            // Optional: implement NMS here

            return detections;
        }

        // Simple detection container
        public static class Detection {
            public final float x1, y1, x2, y2;
            public final String className;
            public final float confidence;

            public Detection(float x1, float y1, float x2, float y2, String className, float confidence) {
                this.x1 = x1;
                this.y1 = y1;
                this.x2 = x2;
                this.y2 = y2;
                this.className = className;
                this.confidence = confidence;
            }
        }
    }
}
