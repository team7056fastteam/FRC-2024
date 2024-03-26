package frc.robot.Common;

import frc.robot.Robot;

public class Line {

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
    Point RobotXY(){
        return new Point(Robot.getPose().getX(), Robot.getPose().getY(), Robot.getPose().getRotation().getRadians());
    }
}
