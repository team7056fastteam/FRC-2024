package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningPidController = new PIDController(0.3, 0.2, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder.setPosition(0);
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getAbsolutePosition().getValue());
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        }
        
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));
        double turnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] state", Math.toDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Encoder Value[" + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValue());

        turningMotor.set(turnSpeed);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }
    public void setDesiredStateUnrestricted(SwerveModuleState state) {
        
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));
        double turnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        turningMotor.set(turnSpeed);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }
}