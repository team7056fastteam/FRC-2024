package frc.robot.Common;

public class Point {
    double x=0, y=0 , radians=0;
    public Point(double x, double y, double radians){
        this.x = x;
        this.y = y;
        this.radians = radians;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getRadians(){
        return radians;
    }
    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setRadians(double radians){
        this.radians = radians;
    }
    public void set(Point point){
        this.x = point.getX();
        this.y = point.getY();
        this.radians = point.getRadians();
    }
    public void setDegrees(double degrees){
        this.radians = Math.toRadians(degrees);
    }
}
