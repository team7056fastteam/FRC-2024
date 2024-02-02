package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends FastCommand{
    private SwerveSubsystem _drive = Robot.getSwerveInstance();
    private double driveX, driveY, driveZ;

    public DriveCommand(double driveX, double driveY, double driveZ){
        this.driveX = driveX;
        this.driveY = driveY;
        this.driveZ = driveZ;
    }

    @Override
    public void init() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, Robot.getGyroscopeRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        _drive.setModuleStates(moduleStates);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return true;
    }

    @Override
    public void end() {}
    
}
