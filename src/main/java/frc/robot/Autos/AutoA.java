package frc.robot.Autos;

import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoA {
    private static Robot _robot = new Robot();

    public static ChassisSpeeds targetChassisSpeeds;
    public static double armAngle_;
    public static double wristAngle_;
    public static double extenderPower_;
    public static double grabberPower_;
    public static boolean armEnabled;
    
    public static void runAutonomousA(double time){
        // 
        if(time > 0 && time < 0.01){
            extenderPower_ = 0;
            wristAngle_ = -80;
            armEnabled = false;
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
             0, _robot.getGyroscopeRotation2d());
        }
        else if(time > 0 && time < .2){
            grabberPower_ = 0.6;
        }
        else if(time > .5 && time < 1){
            armEnabled = true;
            armAngle_ = 95;
        }
        else if(time > 1 && time < 3.5){
            extenderPower_ = -0.7;
        }
        else if(time > 3.5 && time < 4.5){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.5, 0, 
             0, _robot.getGyroscopeRotation2d());
        }
        else if(time > 4.5 && time < 5){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            0, _robot.getGyroscopeRotation2d());
            grabberPower_ = -0.5;
        }
        else{
            grabberPower_ = 0;
            extenderPower_ = 0;
        }
    }
}
