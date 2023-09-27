package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {

    //sets the constants for each module
    private final static SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final static SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final static SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final static SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    ChassisSpeeds targetChassisSpeeds;
    double driveX;
    double driveY;
    double driveZ;
    PIDController xpid = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController ypid = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController zpid = new PIDController(1, 0, 0);

    public SwerveSubsystem() {}

    //sets the states for each module
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    //sets a unrestricted state to each module allows for locking. If you are confused by why this is hit me up.
    public void setModuleStatesUnrestricted(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredStateUnrestricted(desiredStates[0]);
        frontRight.setDesiredStateUnrestricted(desiredStates[1]);
        backLeft.setDesiredStateUnrestricted(desiredStates[2]);
        backRight.setDesiredStateUnrestricted(desiredStates[3]);
    }

    public ChassisSpeeds GoTo(double tx, double ty, double tangle, Pose2d pose){
        double x =  pose.getX();
        double y =  pose.getY();
        Rotation2d gyro = pose.getRotation();
        driveX = xpid.calculate(x, tx) * AutoConstants.kMaxSpeedMetersPerSecond;
        driveY = ypid.calculate(y, ty) * AutoConstants.kMaxSpeedMetersPerSecond;
        zpid.enableContinuousInput(0, 360);
        driveZ = (zpid.calculate(gyro.getDegrees(), tangle)/360) * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
        
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, gyro);
        return targetChassisSpeeds;
    }
    public Boolean GoToComplete(double tx, double ty, double tangle, Pose2d pose){
        double errorX = Math.abs(tx - pose.getX());
        double errorY = Math.abs(ty - pose.getY());
        double errorAngle = Math.abs(tangle - pose.getRotation().getDegrees());
        
        if(errorX > .25 && errorY > .25 && errorAngle > 1){
            return true;
        }
        else{
            return false;
        }
    }
}