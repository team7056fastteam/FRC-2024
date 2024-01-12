package frc.robot.Autos;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoA {
    private static Robot _robot = new Robot();

    public static ChassisSpeeds targetChassisSpeeds;
    public static double ingestPower_ , shooterPower_ ;

    static ProfiledPIDController xController = new ProfiledPIDController(0.4, 0, 0,AutoConstants.kPosControllerConstraints);
    static ProfiledPIDController yController = new ProfiledPIDController(0.4, 0, 0,AutoConstants.kPosControllerConstraints);
    static ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,AutoConstants.kThetaControllerConstraints);

    static int selectedPath, selectedPoint;
    static double selectedX, selectedY, selectedH, xPower, yPower, hPower;

    

    static Boolean kStopPath =  false;

    public static double[][] path0 = {{0,5,Math.toRadians(0)},{0,10,Math.toRadians(0)}};

    public static double[][] path1 = {{0,0,0},{0,0,0}};

    public static double[][][] paths = {path0, path1};
    
    public static void runAutonomousA(Pose2d currentPose){
        runpath(currentPose);

        SmartDashboard.putNumber("SelectedX", selectedX);
        SmartDashboard.putNumber("SelectedY", selectedY);
        SmartDashboard.putNumber("SelectedH", selectedH);
        SmartDashboard.putNumber("xPower", drivePower(xPower));
        SmartDashboard.putNumber("yPower", yPower);
        SmartDashboard.putNumber("SelectedPoint", selectedPoint);
        SmartDashboard.putBoolean("kStopPath", kStopPath);
    }

    static void runpath(Pose2d currentPose){
        selectedX = paths[selectedPath][selectedPoint][0];
        selectedY = paths[selectedPath][selectedPoint][1];
        selectedH = paths[selectedPath][selectedPoint][2];

        double kX = currentPose.getX();
        double kY = currentPose.getY();
        double kH = currentPose.getRotation().getRadians();

        double error = Math.sqrt((Math.pow((selectedX - kX),2) + Math.pow((selectedY - kY),2)));
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("hPower", hPower);
        
        if(!kStopPath){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePower(yPower), 
            -drivePower(xPower), 
            hPower, _robot.getGyroscopeRotation2d());
        }
        else{
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0, 
            0, _robot.getGyroscopeRotation2d());
        }

        if(selectedPoint == (paths[selectedPath].length - 1) ){
            //end point
            xPower = xController.calculate(kX,selectedX);
            yPower = yController.calculate(kY,selectedY);
            hPower = thetaController.calculate(kH, selectedH);
            if(error < 0.125){
                kStopPath = true;
            }
        }
        else{
            //way point
            xPower = xController.calculate(kX,selectedX);
            yPower = yController.calculate(kY,selectedY);
            hPower = thetaController.calculate(kH, selectedH);
            if(error < 0.5){
                advancePoint();
            }
        }
    }

    static void advancePoint(){
        int length = paths[selectedPath].length;

        if (length >= selectedPoint + 1){
            selectedPoint += 1;
        }
    }

    static double drivePower(double power){
        if(Math.abs(power) > AutoConstants.kMaxSpeedMetersPerSecond){
            return Math.signum(power) * AutoConstants.kMaxSpeedMetersPerSecond;
        }
        else{
            return power;
        }
    }
}