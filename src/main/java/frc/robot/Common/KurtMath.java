package frc.robot.Common;

public class KurtMath {
    public static double kurtAngle(double x0, double y0, double x1, double y1){
        double angleRadians = Math.atan2(x1 - x0, y1 - y0);
        return (Math.PI*2) - (angleRadians > 0 ? angleRadians : (angleRadians+2*Math.PI));
        //return angleRadians;
    }
}
