// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Autos.Common.AutoModeRunner;
import frc.robot.Autos.Common.AutoModeSelector;

public class Robot extends TimedRobot {
  //SubSytems
  private SwerveSubsystem _drive;
  //private SpecOps _specOps;
  private NavPod _navpod;

  //Auto
  private AutoModeRunner mAutoModeRunner;
  private AutoModeSelector modeSelector;

  Timer timer = new Timer();
  SlewRateLimiter xLimiter, yLimiter, zLimiter;

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);
  double driveX , driveY , driveZ;
  TrajectoryConfig trajectoryConfig;
  PIDController xController, yController, turnController;
  ProfiledPIDController thetaController, limeX, limeZ;
  HolonomicDriveController controller;

  double kx, ky, kgx, kgy, kgz, gyroRotation;
  Pose2d currentPose;
  double clamp = 0;

  SwerveModuleState[] moduleStates;
  SwerveModuleState[] lockedStates = 
  {
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))
  };
  ChassisSpeeds targetChassisSpeeds;
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx") , ty = table.getEntry("ty") , ta = table.getEntry("ta"), tv = table.getEntry("tv");
  

  boolean lockAuton = false, trackTranslation = false;

  @Override
  public void robotInit() {
    //subsystems
    _navpod = new NavPod();
    _drive = new SwerveSubsystem();
    //_specOps = new SpecOps();

    // Check if the NavPod is connected to RoboRIO
    if (_navpod.isValid()) {
      NavPodConfig config = new NavPodConfig();
      config.cableMountAngle = 0;
      config.fieldOrientedEnabled = true;
      config.initialHeadingAngle = 0;
      config.mountOffsetX = 0;
      config.mountOffsetY = -4;
      config.rotationScaleFactorX = 0.05; // 0.0675
      config.rotationScaleFactorY = 0.05; // 0.02
      config.translationScaleFactor = 0.0008771929824561404; // 0.008567
      _navpod.setConfig(config);

      // Report values to the console
      config = _navpod.getConfig();
      System.err.printf("config.cableMountAngle: %f\n", config.cableMountAngle);
      System.err.printf("config.fieldOrientedEnabled: %b\n", config.fieldOrientedEnabled);
      System.err.printf("config.initialHeadingAngle: %f\n", config.initialHeadingAngle);
      System.err.printf("config.mountOffsetX: %f in\n", config.mountOffsetX);
      System.err.printf("config.mountOffsetY: %f in\n", config.mountOffsetY);
      System.err.printf("config.rotationScaleFactorX: %f\n", config.rotationScaleFactorX);
      System.err.printf("config.rotationScaleFactorY: %f\n", config.rotationScaleFactorY);
      System.err.printf("config.translationScaleFactor: %f\n", config.translationScaleFactor);

      _navpod.resetH(0);
      _navpod.resetXY(0, 0);

      // Keep heading calibrated
      _navpod.setAutoUpdate(0.02, update -> {gyroRotation = update.h; kx = update.x; ky = update.y; kgx = update.gx; kgy = update.gy; kgz = update.gz;});
    }
    trajectoryConfig = new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      .setKinematics(DriveConstants.kDriveKinematics);

    thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(0, Math.PI *2);
    turnController = new PIDController(28,0,3);
    controller = new HolonomicDriveController(xController, yController, thetaController);

    limeX = new ProfiledPIDController(0.1, 0, 0,AutoConstants.kPosControllerConstraints);
    limeZ = new ProfiledPIDController(0.1, 0, 0.1,AutoConstants.kThetaControllerConstraints);

    //Prevents stick stabbing I might need to adjust the zlimiter
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  }

  @Override
  public void robotPeriodic() {
    currentPose = new Pose2d(new Translation2d(-kx, -ky), getGyroscopeRotation2d());

    SmartDashboard.putString("Robot Location", currentPose.toString());
    SmartDashboard.putString("Navpod Gravity Vectors", "GX" + kgx + "GY" + kgy + "GZ" + kgz);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double tag = tv.getDouble(0.0);
    SmartDashboard.putNumber("LimeLightX", x);
    SmartDashboard.putNumber("LimeLightY", y);
    SmartDashboard.putNumber("LimeLightArea", area);
    SmartDashboard.putNumber("Tag", tag);
  }

  @Override
  public void autonomousInit() {
    // Zero robot position
    _navpod.resetXY(0, 0);
    if(modeSelector.getAutoMode() != null){
      mAutoModeRunner.start();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    _navpod.resetXY(0, 0);

    setLimelightCamera(true);
    setLimelight(false);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Driver

    //Speed adjuster
    double xT;
    if (driver.getLeftBumper()){xT = 1.2;}else{xT = 0.45;}
  
    driveX = (driver.getRawAxis(1) * xT * -1);
    driveY = (driver.getRawAxis(0) * xT * -1);
    driveZ = (driver.getRawAxis(4) * -1);

    //apply deadband
    driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
    driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
    driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;

    //smoother
    driveX = xLimiter.calculate(driveX) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    driveY = yLimiter.calculate(driveY) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    driveZ = zLimiter.calculate(driveZ) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, getGyroscopeRotation2d());
    
    if (driver.getRawAxis(2) > 0.1) {
      //lock
      _drive.setModuleStatesUnrestricted(lockedStates);

    } else if(driver.getRawAxis(3) > 0.1){
      // double ModDriveZ = turnController.calculate(getGyroscopeRotation()/180, 1);
      // if(ModDriveZ > 0){
      //   clamp = Math.min(ModDriveZ, 2.5);
      // }
      // else{
      //   clamp = Math.max(ModDriveZ, -2.5);
      // }
      // SmartDashboard.putNumber("ModDriveZ", clamp);
      // ChassisSpeeds modifiedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, clamp, getGyroscopeRotation2d());
      // moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(modifiedChassisSpeeds);
      // _drive.setModuleStates(moduleStates);
    }
    else if(driver.getYButton()){
      //Robot Centric
      // ChassisSpeeds modifiedChassisSpeeds =  new ChassisSpeeds(driveX, driveY, driveZ);
      // moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(modifiedChassisSpeeds);
      // _drive.setModuleStates(moduleStates);
      double y,z;
      if(trackTranslation){
        y = limeX.calculate(tx.getDouble(0.0));
        z = driveZ;
      }
      else{
        y = driveY;
        z = limeZ.calculate(tx.getDouble(0.0));
      }
      ChassisSpeeds modifiedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, y, z, getGyroscopeRotation2d());
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(modifiedChassisSpeeds);
      if(tv.getDouble(0.0) > 0){
        _drive.setModuleStates(moduleStates);
      }
      else{
        _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroscopeRotation2d())));
      }
    }
    else if(driver.getRightBumper()){
      // double angle;
      // if(gyroRotation > 180){
      //   angle = gyroRotation - 360;
      // }
      // else{
      //   angle = gyroRotation;
      // }
      // double ModDriveZ = turnController.calculate(angle/180, 0);
      // if(ModDriveZ > 0){
      //   clamp = Math.min(ModDriveZ, 2.5);
      // }
      // else{
      //   clamp = Math.max(ModDriveZ, -2.5);
      // }
      // SmartDashboard.putNumber("ModDriveZ", clamp);
      // ChassisSpeeds modifiedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, clamp, getGyroscopeRotation2d());
      // moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(modifiedChassisSpeeds);
      // _drive.setModuleStates(moduleStates);
    }
    else{
      //field Centric
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      _drive.setModuleStates(moduleStates);
    }

    //Reset angle of robot to zero
    if(driver.getAButton()){
      _navpod.resetH(0);
      _navpod.resetXY(0, 0);
    }

    //Operator

    //Extender
  //   if(operator.getRawAxis(2) > 0.1 && !_specOps.getLimitSwitchIn()){
  //     _specOps.extendMotorPower(1);
  //   }
  //   else{
  //     if(operator.getRawAxis(3) > 0.1 && _specOps.extendMotorRotLimit()){
  //       _specOps.extendMotorPower(-1);
  //     }
  //     else{
  //       _specOps.extendMotorPower(0);
  //     }
  //   }

  //   //Grabber
  //   if(operator.getAButton() && _specOps.grabberMotorCoderget() > 50){
  //     _specOps.grabberMotorPower(-0.5);
  //   }
  //   else if(operator.getBButton()){
  //     _specOps.grabberMotorPower(0.8);
  //   }
  //   else{
  //     _specOps.grabberMotorPower(0);
  //   }

  //   //Manual Adjustment of Arm Angle
  //   if(operator.getRawAxis(1) < -0.15 && armAngle < 110){
  //     armAngle = armAngle + 0.5;
  //   }
  //   else if(operator.getRawAxis(1) > 0.15 && armAngle > 7){
  //     armAngle = armAngle - 0.5;
  //   }
  //   _specOps.armMotorPosition(armAngle);
  }

  public Rotation2d getGyroscopeRotation2d() {
      return Rotation2d.fromDegrees(gyroRotation);
  }

  /** This function returns the degrees gyro heading */
  public double getGyroscopeRotation() {
      return gyroRotation;
  }

  public void setLimelight(boolean mode) {
    if (mode == true) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    } else {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
  }
  public void setLimelightCamera(boolean mode) {
    if (mode == true) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    } else {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
  }

  public void stop() {
    _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates
    (ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroscopeRotation2d())));
  }

  @Override
  public void disabledInit() {
    stop();
    setLimelight(false);
  }

  @Override
  public void disabledPeriodic() {
    mAutoModeRunner.setAuto(modeSelector.getAutoMode());
  }

  public Pose2d getPose(){
    return currentPose;
  }
}
