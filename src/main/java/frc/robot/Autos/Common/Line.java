package frc.robot.Autos.Common;

import frc.robot.Robot;

public class Line {
    //public enum LineType{Straight, Spline};
    //public LineType type = LineType.Straight;

    Point point0 = new Point(0,0,0);
    Point point1 = new Point(0,0,0);
    double distance = 0;

    public Line(Point point0, Point point1){
        this.point0 = point0;
        this.point1 = point1;
        //this.type = type;

        //if(type == LineType.Straight){
            distance = Math.sqrt(Math.pow((point1.getY() - point0.getY()), 2) + Math.pow((point1.getX() - point0.getX()), 2));
        //}
        //else{
            
       // }
    }
    public double getDistance(){
        return distance;
    }
    public double getHeading(){
        return point1.getH();
    }

    public Point LookAhead(double amount){
        //if(type == LineType.Straight){
        return new Point(getClosestPoint().getX() + amount, getXfromY(getClosestPoint().getX() + amount),0);
        //}
    }

    double DistanceRobotToPoint(Point point)
	{
		return Math.sqrt(Math.pow((Robot.getPose().getY() - point.getY()), 2) + Math.pow((Robot.getPose().getX() - point.getX()), 2));
	}
    Point getClosestPoint(){
        if((point1.getX() == point0.getX())){
            return new Point(point0.getX(),Robot.getPose().getY(),0);
        }
        return new Point(interceptX(point0, point1, new Point(Robot.getPose().getX(),Robot.getPose().getY(),0))
        ,getYfromX(interceptX(point0, point1, new Point(Robot.getPose().getX(),Robot.getPose().getY(),0))),0);
    }
    double getYfromX(double x){
        double m = (point1.getY() - point0.getY())/(point1.getX() - point0.getX());
        return (m)*(x - point0.getX()) + point0.getY();
    }
    double getXfromY(double y){
        double m = (point1.getY() - point0.getY())/(point1.getX() - point0.getX());
        return ((y-point0.getY())/m) + point0.getX();
    }
    double interceptX(Point point0, Point point1, Point point2){
        double m = (point1.getY() - point0.getY())/(point1.getX() - point0.getX());
        double mSquared = m*m;
        return ((mSquared*point0.getX())-(m*point0.getY())+point2.getX()+(m*point2.getY()))/(1+mSquared);
    }
}
