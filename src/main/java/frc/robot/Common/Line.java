package frc.robot.Common;

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

        distance = Math.sqrt(Math.pow((point1.getY() - point0.getY()), 2) + Math.pow((point1.getX() - point0.getX()), 2));
    }
    public double getDistance(){
        return distance;
    }
    public double getHeading(){
        return point1.getRadians();
    }

    public Point LookAhead(double amount){
        return new Point(getClosestPoint().getX() + amount, getClosestPoint().getY() + amount,0);
    }

    double DistanceRobotToPoint(Point point)
	{
		return Math.sqrt(Math.pow((RobotXY().getY() - point.getY()), 2) + Math.pow((RobotXY().getX() - point.getX()), 2));
	}
    Point getClosestPoint(){
        if(point0.getX()==point1.getX()){
            return new Point(point0.getX(), RobotXY().getY(), 0);
        }
        if(point0.getY()==point1.getY()){
            return new Point(RobotXY().getX(), point0.getY(), 0);
        }
        double m1 = (point1.getY()-point0.getY())/(point1.getX()-point0.getX());
        double m2 = -1/m1;
        double x = (m1*point0.getX()-m2*RobotXY().getX()+RobotXY().getY()-point0.getY()) / (m1-m2);
        double y = m2*(x-RobotXY().getX())+RobotXY().getY();
        return new Point(x, y, 0);
    }
    Point RobotXY(){
        return new Point(Robot.getPose().getX(), Robot.getPose().getY(), Robot.getPose().getRotation().getRadians());
    }
}
