package frc.robot.subsystems.Specops;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Specops;

public class Pitchinator extends SubsystemBase{
    public enum PitchState{kIdle, kPitching}
    PitchState state = PitchState.kIdle;

    private Shooter _shooter = Robot.getShooterInstance();

    CANSparkMax PitchingMotor;
    CANCoder pitchCoder;

    double pitch = 0;

    PIDController pitchControl;

    public Pitchinator(){
        PitchingMotor = new CANSparkMax(Specops.kPitchMotor, MotorType.kBrushless);
        pitchCoder = new CANCoder(Specops.kPitchCoder);
        pitchControl = new PIDController(Specops.kPPitch, 0, 0);
    }
    public void setState(PitchState state){
        this.state = state;
        switch(this.state){
            case kIdle:
                setPitchSpeed(0);
                break;
            case kPitching:
                pitch = _shooter.pitch;
                setPitchSpeed(pitchAngle(pitch));
                break;
        }
    }
    double pitchAngle(double angle){
        return pitchControl.calculate(pitchCoder.getAbsolutePosition(),angle);
    }
    void setPitchSpeed(double speed){
        PitchingMotor.set(speed);
    }
}
