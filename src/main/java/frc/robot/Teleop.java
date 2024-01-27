package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.Common.ControllerFunction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Specops.Shooter;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class Teleop {
    private SwerveSubsystem _drive;
    private Robot _robot;
    private Shooter _shoot;
    
    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get = new ControllerFunction(driver, operator);
    double xT, driveX, driveY, driveZ;

    public enum DriveMode{fieldOriented, robotOriented, Targeting, RotationLock, Locked}
    DriveMode mode = DriveMode.fieldOriented;

    SwerveModuleState[] lockedStates = 
    {
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))
    };

    SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    SlewRateLimiter zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    PIDController theta = new PIDController(AutoConstants.kPTargetController, 0, 0);
    
    public Teleop(Robot _robot, SwerveSubsystem _drive, Shooter _shoot){
        this._robot = _robot;
        this._drive =_drive;
        this._shoot = _shoot;
    }

    public void Driver(){
        if(get.speedAdjustment()){xT = 1.2;}else{xT = 0.45;}

        if(get.lockWheels()){mode = DriveMode.Locked;}
        else if(get.robotOriented()){mode = DriveMode.robotOriented;}
        else if(get.Target()){mode = DriveMode.Targeting;}
        else{mode = DriveMode.fieldOriented;}

        if(get.Reset()){
            _robot.resetH();
            _robot.resetXY();
        }

        driveX = get.driverX() * xT;
        driveY = get.driverY() * xT;
        driveZ = get.driverZ();

        //apply deadband
        driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
        driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;
        //smoother
        driveX = xLimiter.calculate(driveX) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        driveY = yLimiter.calculate(driveY) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        driveZ = zLimiter.calculate(driveZ) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        switch(mode){
            case fieldOriented:
                runChassis(driveX, driveY, driveZ);
                _robot.setLimelight(false);
                _robot.setLimelightCamera(false);
                _shoot.setSolutionState(shooterState.kIdle);
                break;
            case robotOriented:
                runRobotOrientedChassis(driveX, driveY, driveZ);
                _robot.setLimelight(false);
                _robot.setLimelightCamera(false);
                _shoot.setSolutionState(shooterState.kIdle);
                break;
            case Targeting:
                _robot.setLimelight(true);
                _robot.setLimelightCamera(true);
                _shoot.dataInSolution(_robot.getPose(), _robot.getTX(), _robot.getTA());
                _shoot.setSolutionState(shooterState.kTarget);
                double z = theta.calculate(_shoot.getYaw());
                runChassis(driveX, driveY, z);
                break;
            case RotationLock:
                break;
            case Locked:
                _drive.setModuleStatesUnrestricted(lockedStates);
                break;
        }
    }
    public void Operator(){

    }
    void runChassis(double driveX, double driveY, double driveZ){
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, _robot.getGyroscopeRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        _drive.setModuleStates(moduleStates);
    }
    void runRobotOrientedChassis(double driveX, double driveY, double driveZ){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(driveX, driveY, driveZ);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        _drive.setModuleStates(moduleStates);
    }

    public void Dashboard(){
        SmartDashboard.putString("TeleOpState", mode.toString());
    }
}
