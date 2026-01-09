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
    private static double minDist;
    private static double maxDist;

    public static void init(Context context) {
        try {
            AssetManager assetManager = context.getAssets();
            double[][] velocityData;
            double[][] rampData;
            try (InputStream vIn = assetManager.open("robotics-data.csv")) {
                double[][] dat = readRoboticsColumnsFromCSV(vIn);
                velocityData = new double[][]{dat[0], dat[1]};
                rampData = new double[][]{dat[0], dat[2]};
            }
            minDist = velocityData[0][0];
            maxDist = velocityData[0][velocityData[0].length - 1];

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

    public static double clampDist(double dist) {
        return Math.max(minDist, Math.min(maxDist, dist));
    }

    public static double getVelocity(double distance) {
        ensureInit();
        distance = clampDist(distance);
        return velocityFunction.value(distance);
    }

    public static double getRampAngle(double distance) {
        ensureInit();
        distance = clampDist(distance);
        return rampFunction.value(distance);
    }

    public static double[][] readRoboticsColumnsFromCSV(InputStream inputStream) throws IOException {
        List<Double> ds = new ArrayList<>();
        List<Double> vs = new ArrayList<>();
        List<Double> as = new ArrayList<>();

        try (BufferedReader buff = new BufferedReader(new InputStreamReader(inputStream))) {
            String line;
            while ((line = buff.readLine()) != null) {
                if (line.trim().isEmpty()) continue;

                String[] parts = line.split(",");
                if (parts.length < 3) continue;

                try {
                    ds.add(Double.parseDouble(parts[0]));
                    vs.add(Double.parseDouble(parts[1]));
                    as.add(Double.parseDouble(parts[2]));
                } catch (NumberFormatException ignored) {
                    // this means that the data it tried to parse wasn't numerical, most likely a header or malformed row
                }
            }
        }

        int n = ds.size();
        double[] dArr = new double[n];
        double[] vArr = new double[n];
        double[] aArr = new double[n];

        List<Integer> idx = new ArrayList<>();
        for (int i = 0; i < n; i++) idx.add(i);
        idx.sort(Comparator.comparingDouble(ds::get));

        for (int i = 0; i < n; i++) {
            vArr[i] = vs.get(idx.get(i));
            aArr[i] = as.get(idx.get(i));
            dArr[i] = ds.get(idx.get(i));
        }

        return new double[][]{dArr, vArr, aArr};
    }
}
