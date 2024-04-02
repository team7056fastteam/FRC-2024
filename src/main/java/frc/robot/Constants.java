package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants {
        //wheel diameter in inches
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        //look these numbers up
        public static final double kDriveMotorGearRatio = 1 / 5.14;
        public static final double kTurningMotorGearRatio = 1 / 18;

        //these convert thoose numbers into positon and velocity
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }
    public static final class DriveConstants {
        //measure left to right wheel
        public static final double kTrackWidth = 0.5;

        //measure front to back wheel
        public static final double kWheelBase = 0.5;

        //should be same if you have a square robot which is typically what you want

        public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

        //drive motors port ids
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 4;
        
        //turn motors port ids
        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        //if the wheels are turning forever they flip the corresponding value
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //to test this put robot up so wheels aren't touching ground and if you put the stick all the way forward they should all be driving in the forward direction
        //if not they adjust this value
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        //abs encoders ids
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        //don't think this needs to be adjusted
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //adjust wheel offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-35.03528);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(125.8048);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(92.72448);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(43.06644);

        //these are the physical max of the motor. Look up the values for these.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6.03504;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //adjust the divisor closer to 1 but never past if you want more speed
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;

        //adjust the divisor closer to 1 but never past if you want faster turning
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 3.5;

        //adjust these values for faster acceleration during teleOp
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;

        //adjust this value if your robot is moving without you touching the sticks. the older controller the more this number typically is
        //you probally want to replace controllers after two seasons or if the stick drift is too high for preicous movement of robot
        public static final double kDeadband = 0.07;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kFastMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.07; //0.4
        public static final double kPYController = 0.07; //0.4
        public static final double kIXController = 0.000; //0.0125
        public static final double kIYController = 0.000; //0.0125
        
        public static final double kPThetaController1 = 7;
        public static final double kPThetaController = 2; //3
        public static final double kPThetaController0 = 2;
        public static final double kPTargetController = 0.075;
        public static final double diagonalController = 0.0125;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final TrapezoidProfile.Constraints kPosControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
    }
    public static final class Specops {
        //Motors
        public static final int kIngestMotor = 9;
        public static final int kKurtinatorMotor = 10;
        public static final int kShooterMotorBottom = 12;
        public static final int kShooterMotorTop = 11;
        public static final int kWinchMotor = 14;
        public static final int kPivotMotor = 15;
        public static final int kSlapperMotor = 13;

        public static final int kPitchCoder = 6;

        public static final int kLeftLimit = 0;
        public static final int kRightLimit = 1;

        public static final int kHighTopRPM = 5000;
        public static final int kHighBottomRPM = 5000;

        public static final int kMidTopRPM = 4000;
        public static final int kMidBottomRPM = 4000;

        public static final int kLowTopRPM = 1800;
        public static final int kLowBottomRPM = 2600;
        //rpm control
        public static final double kPTOP = 0.000008;
        public static final double kPBOTTOM = 0.000008;

        public static final double kAmpPTOP = 0.000006;
        public static final double kAmpPBOTTOM = 0.000006;

        public static final double kPPivot = 0.02;
        public static final double kIPivot = 0.025;

        public static final double kIngestForwardSpeed = 1;
        public static final double kIngestReversedSpeed = -1;

        public static final double kKurtinatorForwardSpeed = 0.6;
        public static final double kKurtinatorExperimentalForwardSpeed = 0.8;
        public static final double kKurtinatorReversedSpeed = -1;

        public static final double kClimberForwardSpeed = 1;
        public static final double kClimberReversedSpeed = -1;

        public static final double kSlappForwardSpeed = -0.1;
        public static final double kSlappReducedForwardSpeed = -0.05;
        public static final double kSlappReversedSpeed = 0.1;
        public static final double kSlappReducedReversedSpeed = 0.05;
        public static final double kSlappAmpLimit = 23;
    }

    public static final class FieldLayout {
        public static final Pose2d blueStartingLeft = new Pose2d(32.66,0, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d blueStartingRight = new Pose2d(143.66,0, new Rotation2d(Math.toRadians(0)));

        public static final double[] blueFirstNote = {35,100,28,3};
        public static final double[] blueSecondNote = {93,100,0,3};
        public static final double[] blueThirdNote = {150,100,337,3};

        public static final double[] blueFirstNoteLong = {13,310,0,3};
        public static final double[] blueSecondNoteLong = {75.5,310,0,5.5};
        public static final double[] blueThirdNoteLong = {138,310,0,3};
        public static final double[] blueFourthNoteLong = {205.5,310,0,3};
        public static final double[] blueFifthNoteLong = {263,310,0,3};

        public static final double[] blueStageLeft = {112,155,356,3};
        public static final double[] blueLeftWingShot = {63,155,12,3};
        public static final double[] blueShootingLeft = {39,70.13,30.63,3};
        public static final double[] blueShootingRight = {188,100,0,3};
        //72.9394602
    }
    
    public static NavPodConfig getNavPodConfig(){
        NavPodConfig config = new NavPodConfig();
        config.cableMountAngle = 270;
        config.fieldOrientedEnabled = true;
        config.initialHeadingAngle = 0;
        config.mountOffsetX = 0;
        config.mountOffsetY = 0;
        config.rotationScaleFactorX = 0.0; // 0.0675
        config.rotationScaleFactorY = 0.0; // 0.02
        config.translationScaleFactor = 0.0077706795; // 0.008567
        return config;
    }
}
