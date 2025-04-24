package org.firstinspires.ftc.teamcode.season.utils;

import com.qualcomm.robotcore.util.Range;

public class Utils {
    public static class Unwrap {
        double lastAngle = 0;

        double update(double angle) {
            double delta = angle - lastAngle;
            if (delta > 180) {
                angle -= 360;
            } else if (delta < -180) {
                angle += 360;
            }
            lastAngle = angle;
            return angle;
        }
    }

    public static double signRoot(double x) {
        return Math.sqrt(Math.abs(x)) * Math.signum(x);
    }

    public static double signSquare(double x) {
        return x * x * Math.signum(x);
    }

    public static double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if (delta > 180) {
            delta = delta - 360;
        } else if (delta < -180) {
            delta = 360 + delta;
        }
        return delta;
    }

    public static double map(double v, double minIn, double maxIn, double minOut, double maxOut) {
        return Range.clip(((v - minIn) / (maxIn - minIn) * (maxOut - minOut) + minOut), Math.min(minOut, maxOut), Math.max(minOut, maxOut));
    }

}
