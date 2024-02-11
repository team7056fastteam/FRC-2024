package frc.robot.subsystems.Specops;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Specops;

public class Pitchinator extends SubsystemBase{
    public enum PitchState{kIdle, kPitching}
    PitchState state = PitchState.kIdle;

    CANSparkMax PitchingMotor;
    CANcoder pitchCoder;

    double pitch = 0;

    PIDController pitchControl;

    public Pitchinator(){
        PitchingMotor = new CANSparkMax(Specops.kPitchCoder, MotorType.kBrushless);
        pitchCoder = new CANcoder(Specops.kPitchCoder);
        pitchControl = new PIDController(Specops.kPPitch, 0, 0);
    }
    public void setState(PitchState state){
        this.state = state;
        switch(this.state){
            case kIdle:
                setPitchSpeed(0);
                break;
            case kPitching:
                pitch = Robot._shooter.pitch;
                setPitchSpeed(pitchAngle(pitch));
                break;
        }
    }
    double pitchAngle(double angle){
        return pitchControl.calculate(pitchCoder.getAbsolutePosition().getValueAsDouble(),angle);
    }
    void setPitchSpeed(double speed){
        PitchingMotor.set(speed);
    }
}
