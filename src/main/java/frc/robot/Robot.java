// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Autos.AutoModeSelector;
import frc.robot.Common.AutoModeRunner;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Specops.Climber;
import frc.robot.subsystems.Specops.Ingest;
import frc.robot.subsystems.Specops.Kurtinator;
import frc.robot.subsystems.Specops.NoteState;
import frc.robot.subsystems.Specops.Pivotinator;
import frc.robot.subsystems.Specops.NoteState.noteState;
import frc.robot.subsystems.Specops.Shooter;
import frc.robot.subsystems.Specops.Slapper;
import frc.robot.subsystems.Specops.Climber.ClimbState;

public class Robot extends TimedRobot {
  //Subsytems
  public static SwerveSubsystem _drive;
  public static Shooter _shooter;
  public static Ingest _ingest;
  public static Kurtinator _kurtinator;
  public static Climber _climber;
  public static Slapper _slapper;
  public static Pivotinator _pivot;
  private static NavPod _navpod;

  //Auto
  private AutoModeRunner mAutoModeRunner;
  private AutoModeSelector modeSelector;

  //Misc
  public static NoteState _noteState;
  private Teleop _teleop;
  private static PhotonCamera camera = new PhotonCamera("Kurt");
  private Spark blinkin;

  static double kx;
  static double ky;
  double kgx;
  double kgy;
  double kgz;
  static double gyroRotation;
  static Pose2d currentPose;
  static PhotonPipelineResult result;
  static PhotonTrackedTarget target;

  static Translation2d xY = new Translation2d(60,0);

  static NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry tx = limeLight.getEntry("tx");

  double samplingRate = 100/20;
  double samplingStep = 0;

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "10.70.56.11", 5800);
    PortForwarder.add(1181, "10.70.56.11", 1181);
    PortForwarder.add(1182, "10.70.56.11", 1182);
    PortForwarder.add(1183, "10.70.56.11", 1183);
    PortForwarder.add(1184, "10.70.56.11", 1184);
    
    //subsystems
    _navpod = new NavPod();
    _drive = new SwerveSubsystem();
    _shooter = new Shooter();
    _ingest = new Ingest();
    _kurtinator = new Kurtinator();
    _climber = new Climber();
    _slapper = new Slapper();
    _pivot =  new Pivotinator();
    modeSelector = new AutoModeSelector();
    mAutoModeRunner = new AutoModeRunner();
    _noteState = new NoteState();
    _teleop = new Teleop();
    blinkin = new Spark(0);

    // Check if the NavPod is connected to RoboRIO
    if (_navpod.isValid()) {
      NavPodConfig config = Constants.getNavPodConfig();
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
  }

  @Override
  public void robotPeriodic() {
    currentPose = new Pose2d(new Translation2d(-kx, -ky), getGyroscopeRotation2d());
    if(camera != null){result = camera.getLatestResult();}
    target = getTarget();
    setBlinkin(_noteState.state == noteState.kNote);
    _noteState.run();
    RobotDashboard();
    _shooter.Dashboard();
    _ingest.Dashboard();
    _kurtinator.Dashboard();
    _climber.Dashboard();
    _teleop.Dashboard();
    _slapper.Dashboard();
    _noteState.Dashboard();
    _pivot.Dashboard();
  }

  @Override
  public void autonomousInit() {
    // Zero robot position
    setXY(0,0);
    resetH();
    enableLimeLight(true);
    samplingStep = 0;

    _climber.setState(ClimbState.kIdle); //This was painful to figure out :(

    if(modeSelector.getAutoMode() != null){
      setXY(-modeSelector.getAutoMode().getStartingPose().getX(),-modeSelector.getAutoMode().getStartingPose().getY());
      mAutoModeRunner.start();
    }
    System.out.println("Autonomous Init");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    _climber.run();
    _shooter.run();
    _ingest.run();
    _kurtinator.run();
    _pivot.run();

    samplingStep++;
    if(samplingRate <= samplingStep){
      System.out.println("Auctal Robot Data: " + currentPose.toString() + " PathRunnerData: " + SmartDashboard.getString("PathRunnerData", null)
       + " Power: " + SmartDashboard.getString("PathPower", null));
      samplingStep = 0;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    enableLimeLight(false);
    _teleop.TeleopInit();
    mAutoModeRunner.stop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    _teleop.Driver();
    _teleop.Operator();
    _climber.run();
    _shooter.run();
    _ingest.run();
    _kurtinator.run();
    _pivot.run();
  }

  public static Rotation2d getGyroscopeRotation2d() {
      return Rotation2d.fromDegrees(gyroRotation);
  }

  public static void resetH(){
    _navpod.resetH(0);
  }
  public static void setXY(double x, double y){
    _navpod.resetXY(x,y);
  }

  public void stop() {
    _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates
    (ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroscopeRotation2d())));
  }

  @Override
  public void disabledInit() {
    stop();
  }

  @Override
  public void disabledPeriodic() {
    if(modeSelector.getAutoMode() != null && mAutoModeRunner != null){
      mAutoModeRunner.stop();
      mAutoModeRunner.setAuto(modeSelector.getAutoMode());
    }
  }

  public static Pose2d getPose(){
    return currentPose;
  }
  public static double getTy(){
    if(target == null){return 0;}
    return target.getYaw();
  }
  public static double getTa(){
    return target.getArea();
  }
  public static boolean hasTargets(){
    return result.hasTargets();
  }
  public static double getTx(){
    return tx.getDouble(0.0);
  }

  public static void enableLimeLight(boolean bool){
    if(bool){
      limeLight.getEntry("pipeline").setNumber(0);
    }else{
      limeLight.getEntry("pipeline").setNumber(1);
    }
  }
  void setBlinkin(boolean mode){
    if(mode){
      blinkin.set(0.15);
    }
    else{
      blinkin.set(0.99);
    }
  }
  public static PhotonTrackedTarget getTarget(){
    List<PhotonTrackedTarget> targets = result.getTargets();
    for(PhotonTrackedTarget target0 : targets){
      if(target0.getFiducialId() == 7 || target0.getFiducialId() == 4){
         return target0;
      }
    }
    return null;
  }
  public static double getId(){
    return target != null ? target.getFiducialId() : -1;
  }
  public static void setGoalTranslation(Translation2d d){
    xY = d;
  }
  public static Translation2d getGoalTranslation(){
    return xY;
  }

  void RobotDashboard(){
    SmartDashboard.putString("Robot Location", currentPose.toString());
    SmartDashboard.putString("Navpod Gravity Vectors", "GX" + kgx + "GY" + kgy + "GZ" + kgz);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Id", getId());
    if(getId() > -1){ SmartDashboard.putNumber("Get Yaw", getTy()); }
  }
}
