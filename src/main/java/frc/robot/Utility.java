package frc.robot;

public class Utility {

    /**
     * Quadratic curve function, with the range set to 1
     * 
     * @param x The input value
     * @return The output value
     */
    public static double quadraticCurve(double x) {
        if (x < 0) {
            return -quadraticCurve(-x);
        }
        return quadraticCurve(x, 1);
    }

    /**
     * Quadratic curve function
     * 
     * @param x     The input value
     * @param range The range of the curve
     * @return The output value
     */
    public static double quadraticCurve(double x, double range) {
        if (x < 0) {
            return -quadraticCurve(-x, range);
        }
        return Math.pow(x / range, 2) * range;
    }
}
