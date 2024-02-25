package frc.robot.Autos.Common;

public class Point {
    double x=0, y=0 , degrees =0;
    public Point(double x, double y, double degrees){
        this.x = x;
        this.y = y;
        this.degrees = 0;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getH(){
        return degrees;
    }
}
