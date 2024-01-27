package frc.robot.Autos.Common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;

public class Pathrunner {
    public static ChassisSpeeds targetChassisSpeeds;
    public static double ingestPower_ , shooterPower_ ;

    static PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    static PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    static PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    
    public static int selectedPath, selectedPoint = 0;
    static double xPower, yPower, hPower, oldXPower = 0, oldYPower = 0, highestXPower = 0, highestYPower = 0;
    
    public static Boolean kStopPath = false;
    
    public ChassisSpeeds runpath(Pose2d currentPose, double[][][] paths,int kselectedPath){
        thetaController.enableContinuousInput(0,2 * Math.PI);

        double selectedX = paths[kselectedPath][selectedPoint][0];
        double selectedY = paths[kselectedPath][selectedPoint][1];
        double selectedH = Math.toRadians(paths[kselectedPath][selectedPoint][2]);

        double selectedError = paths[kselectedPath][selectedPoint][3];

        double kX = currentPose.getX();
        double kY = currentPose.getY();
        double kH = currentPose.getRotation().getRadians();

        double error = Math.sqrt((Math.pow((selectedX - kX),2) + Math.pow((selectedY - kY),2)));
        double errorX = Math.abs(selectedX - kX);
        double errorY = Math.abs(selectedY - kY);
        double angle = Math.toDegrees(Math.atan2((selectedX - kX), (selectedY - kY)));

        double readXPower = xController.calculate(kX,selectedX);
        double readYPower = yController.calculate(kY,selectedY);
        hPower = thetaController.calculate(kH, selectedH);

        if(selectedPoint == (paths[selectedPath].length - 1) ){
            //end point
            xPower = readXPower;
            yPower = readYPower;
            if(errorX < selectedError && errorY < selectedError){
                kStopPath = true;
            }
        }
        else{
            //way point
            oldXPower = xPower;
            oldYPower = yPower;
            
            if(Math.abs(drivePower(readXPower)) > Math.abs(oldXPower)){
                highestXPower = drivePower(readXPower);
            }
            if(Math.abs(drivePower(readYPower)) > Math.abs(oldYPower)){
                highestYPower = drivePower(readYPower);
            }
            if(-45 <= angle && angle <= 45 || -180 <= angle && angle <= -135 || 135 <= angle && angle <= 180){
                
                //yPower constant
                xPower = readXPower;
                yPower = highestYPower;
            }
            else{
                //xPower constant
                yPower = readYPower;
                xPower = highestXPower;
            }
            if(error < selectedError){
                advancePoint(paths);
            }
        }
        if(!kStopPath){
            return ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePower(yPower), 
            -drivePower(xPower), 
            hPower, currentPose.getRotation());
        }
        else{
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 
                0, 
                0, currentPose.getRotation());
        }
    }

    static void advancePoint(double[][][] paths){
        int length = paths[selectedPath].length;

        if (length >= selectedPoint + 1){
            selectedPoint += 1;
        }
    }

    static double drivePower(double power) {
        return Math.abs(power) > AutoConstants.kMaxSpeedMetersPerSecond
                ? Math.signum(power) * AutoConstants.kMaxSpeedMetersPerSecond
                : power;
    }
}
