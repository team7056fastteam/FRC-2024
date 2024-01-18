package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.Autos.Common.Pathrunner;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RunPathCommand extends FastCommand{
    ChassisSpeeds targetChassisSpeeds;
    Pose2d currentPose;
    SwerveSubsystem _drive;
    Pathrunner _path;

    double[][][] paths;
    int selectedPath;

    public RunPathCommand(SwerveSubsystem drive, double[][][] mPaths, Pose2d pose, int currentPath){
        _drive = drive;
        currentPose = pose;
        paths = mPaths;
        selectedPath = currentPath;
    }

    @Override
    public void run() {
        while(!isFinished()){
            targetChassisSpeeds = _path.runpath(currentPose,paths, selectedPath);
            _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        }
    }

    @Override
    public Boolean isFinished() {
        if(Pathrunner.kStopPath){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void init() {
        Pathrunner.kStopPath = false;
        Pathrunner.selectedPoint = 0;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
        
    }

    @Override
    public void end() {
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentPose.getRotation());
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
