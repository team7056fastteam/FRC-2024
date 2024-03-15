package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Common.ControllerFunction;
import frc.robot.Common.KurtMath;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.TeleOpActions.*;
import frc.robot.subsystems.Specops.Climber.ClimbState;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;
import frc.robot.subsystems.Specops.Slapper.SlappState;

public class Teleop {
    
    XboxController driver = new XboxController(0);
    XboxController operator = new XboxController(1);

    ControllerFunction get = new ControllerFunction(driver, operator);
    double xT, driveX, driveY, driveZ, tripped, z;

    Translation2d xY = new Translation2d(0,0);

    public enum DriveMode{fieldOriented, noteTargeting, Targeting, Locked}

    DriveMode mode = DriveMode.fieldOriented;

    SwerveModuleState[] lockedStates = 
    {
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
        new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))
    };

    PIDController theta = new PIDController(AutoConstants.kPTargetController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController0, 0, 0);
    PIDController diagonalController = new PIDController(AutoConstants.diagonalController, 0, 0);

    public void TeleopInit(){
        thetaController.enableContinuousInput(0,2 * Math.PI);
    }

    public void Driver(){
        xY = Robot.getGoalTranslation();
        if(get.speedAdjustment()){xT = 1.4;}else{xT = 0.45;}

        if(get.lockWheels()){mode = DriveMode.Locked;}
        else if(get.AngleLock()){mode = DriveMode.noteTargeting;}
        else if(get.robotOriented()){mode = DriveMode.Targeting;}
        else{mode = DriveMode.fieldOriented;}

        get.Button(get.Reset(), new ResetAction());

        driveX = get.driverX() * xT;
        driveY = get.driverY() * xT;
        driveZ = get.speedAdjustment() ? get.driverZ() : get.driverZ() * 1.3;

        //apply deadband
        driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
        driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
        driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

        //apply DriveConstants
        driveX = driveX * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        driveY = driveY * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        driveZ = driveZ * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        switch(mode){
            case fieldOriented:
                Robot._drive.runChassis(driveX, driveY, driveZ);
                break;
            case noteTargeting:
                z = theta.calculate(Robot.getTx(),10);
                if(Robot.getTx() == 0){ z = 0;}
                Robot._drive.runChassis(driveX, driveY, driveZ + z);
                break;
            case Targeting:
                if(Robot.getId() > -1){
                    z = theta.calculate(Robot.getTy());
                }
                else{
                    // z = thetaController.calculate(
                    // Robot.getPose().getRotation().getRadians(),
                    // KurtMath.kurtAngle(60,0,
                    // Robot.getPose().getX(),Robot.getPose().getY())
                    // );
                    z=0;
                }
                Robot._drive.runChassis(driveX, driveY, driveZ + z);
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
