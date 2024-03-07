package frc.robot.Autos.Common;

public class KurtMath {
    public double kurtAngle(double x0, double y0, double x1, double y1){
        double m = Math.atan2(y1-y0, x1-x0);
        return m > 0 ? m : m+360;
    }
}
