package frc.robot.Common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;

public class NewPathRunner {
    int length = 1;
    public int currentLine = 0;
    double distanceLeft = 0;
    public boolean finished = false;
    Line selectedline;
    Point lookAhead;

    double xPower, yPower;

    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    public ChassisSpeeds runPath(NewPath path){
        //length = path.lines.size();
        selectedline = path.lines.get(currentLine);
        distanceLeft = (selectedline.distance - selectedline.DistanceRobotToPoint(selectedline.point0));
        lookAhead = selectedline.LookAhead(5);

        SmartDashboard.putNumber("distanceLeft", distanceLeft);
        SmartDashboard.putString("lookAhead Point", "X: " + lookAhead.getX() + " Y: " + lookAhead.getY());
        SmartDashboard.putString("Closest Point", "X: " + selectedline.getClosestPoint().getX() + " Y: " + selectedline.getClosestPoint().getY());

        if(length == currentLine){
            //endpoint
            if(distanceLeft < 3){
                if(distanceLeft < 8){
                    xPower = xController.calculate(Robot.getPose().getX(), lookAhead.getX());
                    yPower = yController.calculate(Robot.getPose().getY(), lookAhead.getY());
                }
                else{
                    xPower = xController.calculate(Robot.getPose().getX(), selectedline.point1.getX());
                    yPower = yController.calculate(Robot.getPose().getY(), selectedline.point1.getY());
                }
            }
            else{
                finished = true;
            }
        }
        else{
            //waypoint
            if(distanceLeft < 3){
                currentLine ++;
            }
            else{
                xPower = xController.calculate(Robot.getPose().getX(), lookAhead.getX());
                yPower = yController.calculate(Robot.getPose().getY(), lookAhead.getY());
            }
        }
        if(finished){
            return ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Robot.getPose().getRotation());
        }
        else{
            return ChassisSpeeds.fromFieldRelativeSpeeds(drivePower(yPower), -drivePower(xPower),0, Robot.getPose().getRotation());
        }
    }

    static double drivePower(double power) {
        return Math.abs(power) > AutoConstants.kMaxSpeedMetersPerSecond
                ? Math.signum(power) * AutoConstants.kMaxSpeedMetersPerSecond
                : power;
    }
}
