package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.Autos.Common.Pathrunner;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RunPathCommand extends FastCommand{
    SwerveSubsystem _drive = Robot.getSwerveInstance();
    Robot _robot = new Robot();
    Pathrunner _path = new Pathrunner();

    ChassisSpeeds targetChassisSpeeds;
    Pose2d currentPose;

    double[][][] paths;
    int selectedPath;

    public RunPathCommand(double[][][] mPaths, int currentPath){
        paths = mPaths;
        selectedPath = currentPath;
    }

    @Override
    public void run() {
        if(_drive.getPose() != null){
           _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
           _path.runpath(_drive.getPose() ,paths, selectedPath)));
        }
    }

    @Override
    public Boolean isFinished() {
        return Pathrunner.kStopPath;
    }

    @Override
    public void init() {
        Pathrunner.kStopPath = false;
        Pathrunner.selectedPoint = 0;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, _drive.getPose().getRotation());
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        
    }

    @Override
    public void end() {
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, _drive.getPose().getRotation());
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
