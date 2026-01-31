package org.firstinspires.ftc.teamcode.starterbot.interpolation;

import org.apache.commons.math3.analysis.UnivariateFunction;

public class PchipInterpolator implements UnivariateFunction {
    private final double[] x;
    private final double[] y;
    private final double[] d;

    public PchipInterpolator(double[] x, double[] y) {
        this.x = x;
        this.y = y;
        this.d = computeDerivatives(x, y);
    }

    private static double[] computeDerivatives(double[] x, double[] y) {
        int n = x.length;
        double[] d = new double[n];
        double[] delta = new double[n - 1];

        for (int i = 0; i < n - 1; i++) {
            delta[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        }

        d[0] = delta[0];
        d[n - 1] = delta[n - 2];

        for (int i = 1; i < n - 1; i++) {
            if (delta[i - 1] * delta[i] <= 0) {
                d[i] = 0;
            } else {
                double w1 = 2 * (x[i + 1] - x[i]) + (x[i] - x[i - 1]);
                double w2 = (x[i + 1] - x[i]) + 2 * (x[i] - x[i - 1]);
                d[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
            }
        }
        return d;
    }

    public double value(double xi) {
        int i = findInterval(xi);
        double h = x[i + 1] - x[i];
        double t = (xi - x[i]) / h;

        double h00 = (1 + 2 * t) * (1 - t) * (1 - t);
        double h10 = t * (1 - t) * (1 - t);
        double h01 = t * t * (3 - 2 * t);
        double h11 = t * t * (t - 1);

        return h00 * y[i]
                + h10 * h * d[i]
                + h01 * y[i + 1]
                + h11 * h * d[i + 1];
    }

    private int findInterval(double xi) {
        int i = java.util.Arrays.binarySearch(x, xi);
        if (i < 0) i = -i - 2;
        return Math.max(0, Math.min(i, x.length - 2));
    }
}
