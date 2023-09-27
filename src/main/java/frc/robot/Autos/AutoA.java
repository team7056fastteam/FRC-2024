package frc.robot.Autos;

import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoA {
    static ChassisSpeeds targetChassisSpeeds;

    private static Robot _robot = new Robot();

    public static ChassisSpeeds chassisSpeedAutoA(){
        return targetChassisSpeeds;
    }
    public static void runAutonomousA(double time){
        if(time > 0 && time < 1){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(.5, 0, 
            0, _robot.getGyroscopeRotation2d());
        }
        else{
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
            0, _robot.getGyroscopeRotation2d());
        }
    }
}
