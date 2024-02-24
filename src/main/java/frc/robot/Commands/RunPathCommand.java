package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.Autos.Common.Path;
import frc.robot.Autos.Common.Pathrunner;
import frc.robot.Constants.DriveConstants;

public class RunPathCommand extends FastCommand{
    Pathrunner _path = new Pathrunner();

    ChassisSpeeds targetChassisSpeeds;
    Pose2d currentPose;

    Path path;

    public RunPathCommand(Path path){
        this.path = path;
    }

    @Override
    public void run() {
        if(Robot.getPose() != null){
            Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
            _path.runpath(Robot.getPose(), path)));
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
        Pathrunner.highestXPower = 0;
        Pathrunner.highestYPower = 0;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        
    }

    @Override
    public void end() {
        Pathrunner.kStopPath = true;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
