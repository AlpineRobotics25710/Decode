package org.firstinspires.ftc.teamcode.starterbot;

import android.content.Context;
import android.content.res.AssetManager;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.UnivariateInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Interpolator {
    private static PolynomialSplineFunction velocityFunction;
    private static PolynomialSplineFunction rampFunction;

    public static void init(Context context) {
        try {
            AssetManager assetManager = context.getAssets();
            double[][] velocityData;
            double[][] rampData;
            try (InputStream vIn = assetManager.open("robotics-data.csv");
                 InputStream rIn = assetManager.open("robotics-data.csv")) {
                velocityData = readRoboticsColumnsFromCSV(vIn, 1);
                rampData = readRoboticsColumnsFromCSV(rIn, 2);
            }
            assetManager.close();

            UnivariateInterpolator interpolator = new SplineInterpolator();
            velocityFunction = (PolynomialSplineFunction) interpolator.interpolate(velocityData[0], velocityData[1]);
            rampFunction = (PolynomialSplineFunction) interpolator.interpolate(rampData[0], rampData[1]);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static void ensureInit() {
        if (velocityFunction == null || rampFunction == null) {
            throw new IllegalStateException("Interpolator.init() was not called");
        }
    }

    public static double getVelocity(double distance) {
        ensureInit();
        return velocityFunction.value(distance);
    }

    public static double getRampAngle(double distance) {
        ensureInit();
        return rampFunction.value(distance);
    }

    public static double[][] readRoboticsColumnsFromCSV(InputStream inputStream, int yColumnIndex) throws IOException {
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        try (BufferedReader buff = new BufferedReader(new InputStreamReader(inputStream))) {
            String line;
            while ((line = buff.readLine()) != null) {
                if (line.trim().isEmpty()) continue;

                String[] parts = line.split(",");
                if (parts.length <= yColumnIndex) continue;

                try {
                    xs.add(Double.parseDouble(parts[0]));
                    ys.add(Double.parseDouble(parts[yColumnIndex]));
                } catch (NumberFormatException ignored) {
                    // this means that the data it tried to parse wasn't numerical, most likely a header or malformed row
                }
            }
        }

        int n = xs.size();
        double[] xArr = new double[n];
        double[] yArr = new double[n];

        List<Integer> idx = new ArrayList<>();
        for (int i = 0; i < n; i++) idx.add(i);
        idx.sort(Comparator.comparingDouble(xs::get));

        for (int i = 0; i < n; i++) {
            xArr[i] = xs.get(idx.get(i));
            yArr[i] = ys.get(idx.get(i));
        }

        return new double[][]{xArr, yArr};
    }
}
