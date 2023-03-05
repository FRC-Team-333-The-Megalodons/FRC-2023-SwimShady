package frc.robot.utils;

public class RobotMath {
    final static double DEFAULT_EPSILON = 0.1;
    public static boolean isApprox(double a, double b)
    {
        return isApprox(a, b, DEFAULT_EPSILON);
    }
    public static boolean isApprox(double a, double b, double epsilon)
    {
        double delta = Math.abs(a-b);
        return delta <= epsilon;
    }

    public static boolean inRange(double x, double lower, double upper)
    {
        return x > lower && x < upper;
    }
}
