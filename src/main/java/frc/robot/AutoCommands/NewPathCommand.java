package frc.robot.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Common.NewPath;
import frc.robot.Common.NewPathRunner;

public class NewPathCommand extends FastCommand{
    NewPathRunner _path = new NewPathRunner();
    NewPath path;

    public NewPathCommand(NewPath path){
        this.path = path;
    }

    @Override
    public void init() {
        _path.finished = false;
        _path.currentLine = 0;
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }

    @Override
    public void run() {
       if(Robot.getPose() != null){
            Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
            _path.runPath(path)));
        }
    }

    @Override
    public Boolean isFinished() {
        return _path.finished;
    }

    @Override
    public void end() {
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
