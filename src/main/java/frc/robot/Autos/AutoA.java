package frc.robot.Autos;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoA {
    private static Robot _robot = new Robot();

    public static ChassisSpeeds targetChassisSpeeds;
    public static double ingestPower_ , shooterPower_ ;

    static PIDController xController = new PIDController(1, 0, 0);
    public static PIDController yController = new PIDController(1, 0, 0);

    static int selectedPath, selectedPoint;
    static double selectedX, selectedY, selectedH;

    public static double[][] path0 = {{0,0,0},{0,0,0}};

    public static double[][] path1 = {{0,0,0},{0,0,0}};

    public static double[][][] paths = {path0, path1};
    
    public static void runAutonomousA(Pose2d currentPose){
        selectedX = paths[selectedPath][selectedPoint][0];
        selectedY = paths[selectedPath][selectedPoint][1];
        selectedH = paths[selectedPath][selectedPoint][2];

        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePower(xController.calculate(currentPose.getX(),selectedX)), 
            drivePower(yController.calculate(currentPose.getY(),selectedY)), 
             0, _robot.getGyroscopeRotation2d());

        if(paths[selectedPath].length == selectedPoint + 1){
            //less error because it is endpoint
            if(currentPose.getX() - 0.2 <= selectedX && selectedX <= currentPose.getX() + 0.2){
                advancePoint();
            }
        }
        else{
            //more error cause checkpoint
            if(currentPose.getX() - 1.0 <= selectedX && selectedX <= currentPose.getX() + 1.0){
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
    static void advancePath(){
        int length = paths.length;

        if (length >= selectedPath + 1){
            selectedPath += 1;
        }
    }

    static double drivePower(double power){
        if(power > AutoConstants.kMaxSpeedMetersPerSecond){
            return AutoConstants.kMaxSpeedMetersPerSecond;
        }
        else{
            return power;
        }
    }
}