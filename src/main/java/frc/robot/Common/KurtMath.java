package frc.robot.Common;

public class KurtMath {
    public static double kurtAngle(double x0, double y0, double x1, double y1){
        double angleRadians = Math.atan2(x1 - x0, y1 - y0);
        return (Math.PI*2) - (angleRadians > 0 ? angleRadians : (angleRadians+2*Math.PI));
        //return angleRadians;
    }

    public static double[] modifyAngle(double[] point, double newAngle){
        double[] newPoint = {point[0], point[1], newAngle, point[3]};
        return newPoint;
    }

    public static double[] modifyError(double[] point, double newError){
        double[] newPoint = {point[0], point[1], point[2], newError};
        return newPoint;
    }

    public static double[] addXYToPoint(double[] point, double x, double y){
        double[] newPoint = {point[0] + x, point[1] + y, point[2], point[3]};
        return newPoint;
    }

    public static double[] convertToVelocity(double[] point, double velocity, double error){
        double[] newPoint = {point[0], point[1], point[2], error, velocity};
        return newPoint;
    }
}
