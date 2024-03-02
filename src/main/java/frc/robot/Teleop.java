package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.Common.ControllerFunction;
import frc.robot.Commands.TeleOpActions.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Specops.Climber.ClimbState;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;
import frc.robot.subsystems.Specops.Slapper.SlappState;

public class Teleop {
    
    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get = new ControllerFunction(driver, operator);
    double xT, driveX, driveY, driveZ, tripped;

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

    public void Driver(){
        if(get.speedAdjustment()){xT = 1.4;}else{xT = 0.45;}

        if(get.lockWheels()){mode = DriveMode.Locked;}
        //else if(get.robotOriented()){mode = DriveMode.robotOriented;}
        else if(get.Target() || get.robotOriented()){mode = DriveMode.Targeting;}
        else{mode = DriveMode.fieldOriented;}

        get.Button(get.Reset(), new ResetAction());

        driveX = get.driverX() * xT;
        driveY = get.driverY() * xT;
        driveZ = get.speedAdjustment() ? get.driverZ() : get.driverZ() * 1.3;

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
                Robot._drive.runChassis(driveX, driveY, driveZ);
                break;
            case robotOriented:
                Robot._drive.runRobotOrientedChassis(driveX, driveY, driveZ);
                Robot._shooter.setSolutionState(shooterState.kIdle);
                break;
            case Targeting:
                double z = theta.calculate(Robot.GetTX()-2);
                if(Robot.hasTargets()){
                    Robot._drive.runChassis(driveX, driveY, z);
                }
                else{
                    Robot._drive.runChassis(driveX, driveY, driveZ);
                }
                break;
            case RotationLock:
                break;
            case Locked:
                Robot._drive.setModuleStatesUnrestricted(lockedStates);
                break;
        }
    }
    public void Operator(){
        get.Button(get.HighShot(), new ShooterAction(shooterState.kHigh));
        get.Button(get.LowShot(), new ShooterAction(shooterState.kLow));
        get.Button(mode == DriveMode.Targeting, new ShooterAction(shooterState.kTarget));
        get.Button(!get.HighShot() && !get.LowShot() && mode != DriveMode.Targeting, new ShooterAction(shooterState.kIdle));

        get.Button(get.IngestIn(), new IngestAction(IngestState.kForward, KurtinatorState.kRunTilTrip));
        get.Button(get.IngestOut(), new IngestAction(IngestState.kReversed, KurtinatorState.kReversed));
        get.Button(get.Feed(), new IngestAction(IngestState.kIdle, KurtinatorState.kFeed));
        get.Button(!get.IngestIn() && !get.IngestOut() && !get.Feed(), new IngestAction(IngestState.kIdle, KurtinatorState.kIdle));

        get.Button(get.Climb(), new ClimberAction(ClimbState.kClimb));
        get.Button(get.UnClimb(), new ClimberAction(ClimbState.kUnClimb));
        get.Button(!get.Climb() && !get.UnClimb(), new ClimberAction(ClimbState.kIdle));

        get.Button(get.Flipp(), new SlapperAction(SlappState.kSlapp));
        get.Button(get.UnFlipp(), new SlapperAction(SlappState.kUnSlapp));
        get.Button(!get.Flipp() && !get.UnFlipp(), new SlapperAction(SlappState.kIdle));
    }

    public void Dashboard(){
        SmartDashboard.putString("TeleOpState", mode.toString());
    }
}
