package frc.robot.Autos.Common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;

public class NewPathRunner {
    int length = 1;
    public int currentLine = 0;
    double distanceConsumed = 0;
    public boolean finished = false;
    Line selectedline;

    double xPower, yPower;

    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    public ChassisSpeeds runPath(NewPath path){
        //length = path.lines.size();
        selectedline = path.lines.get(currentLine);
        distanceConsumed = (selectedline.distance - selectedline.DistanceRobotToPoint(selectedline.point1));

        if(length == currentLine){
            //endpoint
            if(distanceConsumed < 3){
                xPower = xController.calculate(Robot.getPose().getX(), selectedline.point1.getX());
                yPower = yController.calculate(Robot.getPose().getY(), selectedline.point1.getY());
            }
            else{
                finished = true;
            }
        }
        else{
            //waypoint
            if(distanceConsumed < 3){
                currentLine ++;
            }
            else{
                Point lookAhead = selectedline.LookAhead(2);
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
