package frc.robot.Autos;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoA {
    private static Robot _robot = new Robot();

    public static ChassisSpeeds targetChassisSpeeds;
    public static double ingestPower_ , shooterPower_ ;

    static ProfiledPIDController xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kPosControllerConstraints);
    static ProfiledPIDController yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kPosControllerConstraints);
    static ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    static int selectedPath, selectedPoint;
    static double selectedX, selectedY, selectedH, xPower, yPower, hPower, oldXPower, oldYPower, highestXPower, highestYPower;

    static Boolean kStopPath =  false;
    static Boolean kStop =  false;

    public static double[][] path0 = {{0,5,Math.toRadians(0)},{0,10,Math.toRadians(0)}};

    public static double[][] path1 = {{0,0,0},{0,0,0}};

    public static double[][][] paths = {path0, path1};
    
    public static void runAutonomousA(Pose2d currentPose){
        if(!kStop){

            kStopPath =  false;
            runpath(currentPose, 0);

            sleep(2.5);

            kStopPath =  false;
            runpath(currentPose, 1);
        }
    }

    static void runpath(Pose2d currentPose, int kselectedPath){
        while (kStopPath){
            updateDashboard();

            selectedPath = kselectedPath;

            selectedX = paths[selectedPath][selectedPoint][0];
            selectedY = paths[selectedPath][selectedPoint][1];
            selectedH = paths[selectedPath][selectedPoint][2];

            double kX = currentPose.getX();
            double kY = currentPose.getY();
            double kH = currentPose.getRotation().getRadians();

            //double error = Math.sqrt((Math.pow((selectedX - kX),2) + Math.pow((selectedY - kY),2)));
            double error = Math.hypot((selectedX - kX), (selectedY - kY));
            double angle = Math.toDegrees(Math.atan2((selectedY - kY), (selectedX - kX)));
            SmartDashboard.putNumber("Error", error);
            SmartDashboard.putNumber("Angle", angle);
            
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePower(yPower), 
            -drivePower(xPower), 
            hPower, _robot.getGyroscopeRotation2d());


            if(selectedPoint == (paths[selectedPath].length - 1) ){
                //end point
                xPower = xController.calculate(kX,selectedX);
                yPower = yController.calculate(kY,selectedY);
                hPower = thetaController.calculate(kH, selectedH);
                if(error < 0.20){
                    kStopPath = true;
                }
            }
            else{
                //way point
                oldXPower = xPower;
                oldYPower = yPower;

                xPower = xController.calculate(kX,selectedX);
                yPower = yController.calculate(kY,selectedY);
                hPower = thetaController.calculate(kH, selectedH);

                if(Math.abs(drivePower(xPower)) > Math.abs(oldXPower)){
                    highestXPower = drivePower(xPower);
                }
                if(Math.abs(drivePower(yPower)) > Math.abs(oldYPower)){
                    highestYPower = drivePower(yPower);
                }

                if(angle > 90){
                    //yPower constant
                    xPower = xController.calculate(kX,selectedX);
                    yPower = highestYPower;
                }
                else{
                    //xPower constant
                    yPower = yController.calculate(kY,selectedY);
                    xPower = highestXPower;
                }
                if(error < 0.5){
                    advancePoint();
                }
            }
        }
        if(kStopPath){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0, 
            0, _robot.getGyroscopeRotation2d());
        }
    }

    static void advancePoint(){
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

    static Timer time = new Timer();

    static void sleep(double seconds){
        time.reset();
        time.start();
        while(time.get() < seconds){}
    }

    static void updateDashboard(){
        SmartDashboard.putNumber("SelectedX", selectedX);
        SmartDashboard.putNumber("SelectedY", selectedY);
        SmartDashboard.putNumber("SelectedH", selectedH);

        SmartDashboard.putNumber("xPower", drivePower(xPower));
        SmartDashboard.putNumber("yPower", drivePower(yPower));
        SmartDashboard.putNumber("hPower", hPower);

        SmartDashboard.putBoolean("kStopPath", kStopPath);
    }
}