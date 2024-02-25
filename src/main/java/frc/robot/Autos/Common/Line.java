package frc.robot.Autos.Common;

import frc.robot.Robot;

public class Line {
    public enum LineType{Straight, Spline};
    public LineType type = LineType.Straight;

    Point point0 = new Point(0,0,0);
    Point point1 = new Point(0,0,0);
    double distance = 0;

    public Line(Point point, Point point2, LineType type){
        this.point0 = point;
        this.point1 = point2;
        this.type = type;

        if(type == LineType.Straight){
            distance = Math.sqrt(Math.pow((point2.getY() - point.getY()), 2) + Math.pow((point2.getX() - point.getX()), 2));
        }
        else{
            distance = Math.sqrt(Math.pow((point2.getY() - point.getY()), 2) + Math.pow((point2.getX() - point.getX()), 2));
        }
    }
    public double getDistance(){
        return distance;
    }
    public double getHeading(){
        return point1.getH();
    }

    public Point LookAhead(double amount){
        return new Point(getClosestPoint().getX() + amount, getXfromY(getClosestPoint().getX() + amount),0);
    }

    double DistanceRobotToPoint(Point point)
	{
		return Math.sqrt(Math.pow((Robot.getPose().getY() - point.getY()), 2) + Math.pow((Robot.getPose().getX() - point.getX()), 2));
	}

    // double splineDistance(Point point0, Point point1){
    //     return 0;
    // }
    Point getClosestPoint(){
        return new Point(((Robot.getPose().getX() - getXfromY(Robot.getPose().getY()))/2),((Robot.getPose().getY() - getYfromX(Robot.getPose().getX()))/2),0);
    }
    double getYfromX(double x){
        return ((point1.getY() - point0.getY())/(point1.getX() - point0.getX()))*(x - point0.getX()) + point0.getY();
    }
    double getXfromY(double y){
        return (1/((point1.getY() - point0.getY())/(point1.getX() - point0.getX())))*(y - point0.getX()) + point0.getY();
    }
}
