package frc.robot.Commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;

public class TurnCommand extends FastCommand{
    SwerveSubsystem _drive = Robot.getSwerveInstance();
    ChassisSpeeds targetChassisSpeeds;

    Boolean halfway;

    @Override
    public void init() {
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, _drive.getPose().getRotation());
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        halfway = false;
    }

    @Override
    public void run() {
        targetChassisSpeeds = new ChassisSpeeds(0, 0, 0.8);
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        if(_drive.getPose().getRotation().getRadians() > Math.PI){
            halfway = true;
        }
    }

    @Override
    public Boolean isFinished() {
        if(halfway){
            return 0.07 > _drive.getPose().getRotation().getRadians() || _drive.getPose().getRotation().getRadians() > ((2*Math.PI) -0.07);
        }
        else{
            return false;
        }
    }

    @Override
    public void end() {
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, _drive.getPose().getRotation());
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
