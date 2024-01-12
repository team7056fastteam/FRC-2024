package frc.robot;

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
        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kBackLeftDriveMotorPort = 21;
        public static final int kFrontRightDriveMotorPort = 22;
        public static final int kBackRightDriveMotorPort = 20;
        
        //turn motors port ids
        public static final int kFrontLeftTurningMotorPort = 23;
        public static final int kBackLeftTurningMotorPort = 11;
        public static final int kFrontRightTurningMotorPort = 12;
        public static final int kBackRightTurningMotorPort = 10;

        //if the wheels are turning forever they flip the corresponding value
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //to test this put robot up so wheels aren't touching ground and if you put the stick all the way forward they should all be driving in the forward direction
        //if not they adjust this value
        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = true;

        //abs encoders ids
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 5;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        //don't think this needs to be adjusted
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //adjust wheel offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-90);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-40.5);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(135);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(30);

        //these are the physical max of the motor. Look up the values for these.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6.03504;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //adjust the divisor closer to 1 but never past if you want more speed
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;

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
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.25;
        public static final double kPYController = 0.25;
        public static final double kPThetaController1 = 7;
        public static final double kPThetaController = 5;

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
        public static final int kIngestMotor = 3;
        public static final int kShooterMotor = 5;
        // public static final int kWristMotor = 4;
        // public static final int kArmMotor = 2;
        // //Encoders
        // public static final int kArmCoder = 6;
        // public static final int kExtendMotorCoder = 7;
        // public static final int kGrabberMotorCoder = 8;
        // public static final int kWristMotorCoder = 9;
        // //Limit
        // public static final int kLimitSwitchIn = 0;
        // public static final int kLimitSwitchOut = 1;
        // public static final double kExtendMotorRotLimit = 7.3 * 360;
        // public static final double armSpoolLimit = 30;
    }
}
