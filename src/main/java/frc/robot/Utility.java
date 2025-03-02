package frc.robot;

public class Utility {

    public static double quadraticCurve(double x) {
        if (x < 0) {
            return -quadraticCurve(-x);
        }
        return quadraticCurve(x, 1);
    }

    public static double quadraticCurve(double x, double range) {
        if (x < 0) {
            return -quadraticCurve(-x, range);
        }
        return Math.pow(x / range, 2) * range;
    }
}
