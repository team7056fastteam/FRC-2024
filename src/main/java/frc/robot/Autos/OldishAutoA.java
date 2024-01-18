package frc.robot.Autos;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OldishAutoA {
    private static Robot _robot = new Robot();

    public static ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, _robot.getGyroscopeRotation2d());
    public static double ingestPower_ , shooterPower_ ;

    static PIDController xController = new PIDController(AutoConstants.kPXController, 0.0125, 0);
    static PIDController yController = new PIDController(AutoConstants.kPYController, 0.0125, 0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    public static int selectedPath, selectedPoint = 0;
    static double selectedX, selectedY, selectedH, xPower, yPower, hPower, oldXPower = 0, oldYPower = 0, highestXPower = 0, highestYPower = 0;

    public static Boolean[] kStopPath =  {false,false,false,false,false};
    public static Boolean onetime = false;

    //{x,y,heading,error}
    public static double[][] path0 = {{-0.7,2.0,0,0.2}};

    public static double[][] path1 = {{ -.7,9.2,0,0.2}};

    public static double[][] path2 = {{5.98,5.41,0,0.15}};

    public static double[][][] paths = {path0, path1, path2};
    
    public static void runAutonomousA(Pose2d currentPose){
        if(!kStopPath[0]){
            runpath(currentPose, 0);
        }
        else if(!kStopPath[1]){
            runpath(currentPose, 1);
        }
        else if(!onetime){
            stop();
            Timer.delay(2);
            onetime = true;
        }
        else if(!kStopPath[2]){
            runpath(currentPose, 2);
        }
        else{
            stop();
        }
    }

    static void runpath(Pose2d currentPose, int kselectedPath){
        updateDashboard();
        selectedPath = kselectedPath;
        selectedX = paths[kselectedPath][selectedPoint][0];
        selectedY = paths[kselectedPath][selectedPoint][1];
        selectedH = Math.toRadians(paths[kselectedPath][selectedPoint][2]);
        double selectedError = paths[kselectedPath][selectedPoint][3];

        double kX = currentPose.getX();
        double kY = currentPose.getY();
        double kH = currentPose.getRotation().getRadians();

        double error = Math.sqrt((Math.pow((selectedX - kX),2) + Math.pow((selectedY - kY),2)));
        double errorX = Math.abs(selectedX - kX);
        double errorY = Math.abs(selectedY - kY);
        double angle = Math.toDegrees(Math.atan2((selectedX - kX), (selectedY - kY)));

        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Angle", angle);

        if(!kStopPath[kselectedPath]){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePower(yPower), 
            -drivePower(xPower), 
            hPower, _robot.getGyroscopeRotation2d());
        }
        else{
            stop();
        }

        double readXPower = xController.calculate(kX,selectedX);
        double readYPower = yController.calculate(kY,selectedY);
        hPower = thetaController.calculate(kH, selectedH);

        if(selectedPoint == (paths[selectedPath].length - 1) ){
            //end point
            xPower = readXPower;
            yPower = readYPower;
            if(errorX < selectedError && errorY < selectedError){
                kStopPath[kselectedPath] = true;
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
            // xPower = xController.calculate(kX,selectedX);
            // yPower = yController.calculate(kY,selectedY);
            // hPower = thetaController.calculate(kH, selectedH);
            if(error < selectedError){
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

    static double drivePower(double power) {
        return Math.abs(power) > AutoConstants.kMaxSpeedMetersPerSecond
                ? Math.signum(power) * AutoConstants.kMaxSpeedMetersPerSecond
                : power;
    }

    static void updateDashboard(){
        SmartDashboard.putNumber("SelectedX", selectedX);
        SmartDashboard.putNumber("SelectedY", selectedY);
        SmartDashboard.putNumber("SelectedH", selectedH);

        SmartDashboard.putNumber("xPower", drivePower(xPower));
        SmartDashboard.putNumber("yPower", drivePower(yPower));
        SmartDashboard.putNumber("hPower", hPower);

        SmartDashboard.putNumber("highestXPower", highestXPower);
        SmartDashboard.putNumber("highestYPower", highestYPower);

        SmartDashboard.putBoolean("kStopPath", kStopPath[selectedPath]);
    }
    static void stop(){
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0, 
            0, _robot.getGyroscopeRotation2d());
    }
}