package frc.robot.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.Constants.DriveConstants;

public class DriveCommand extends FastCommand{
    private double driveX, driveY, driveZ;
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] moduleStates;

    public DriveCommand(double driveX, double driveY, double driveZ){
        this.driveX = driveX;
        this.driveY = driveY;
        this.driveZ = driveZ;
    }

    @Override
    public void init() {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, Robot.getGyroscopeRotation2d());
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    }

    @Override
    public void run() {
        Robot._drive.setModuleStates(moduleStates);
    }

    @Override
    public Boolean isFinished() {
        return false;
    }

    @Override
    public void end() {}
    
}
