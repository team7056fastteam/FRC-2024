package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;

public class Climber extends SubsystemBase{
    CANSparkMax climberMotor;
    RelativeEncoder climbCoder;

    public enum ClimbState{kIdle, kClimb, kUnClimb}
    ClimbState state = ClimbState.kIdle;
    
    public Climber(){
        climberMotor = new CANSparkMax(Specops.kWinchMotor, MotorType.kBrushless);
        climbCoder = climberMotor.getEncoder();
        climbCoder.setPosition(0);
        climberMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setState(ClimbState state){
        this.state = state;
        switch(this.state){
            case kIdle:
                setClimberSpeed(0);
                break;
            case kClimb:
                setClimberSpeed(Specops.kClimberForwardSpeed);
                break;
            case kUnClimb:
                //if(climbCoder.getPosition() > 0){
                    setClimberSpeed(Specops.kClimberReversedSpeed);
                // }
                // else{
                //     setClimberSpeed(0);
                // }   
                break;
        }
    }

    void setClimberSpeed(double speed){
        climberMotor.set(speed);
    }

    public void Dashboard(){
        SmartDashboard.putString("Climb State", state.toString());
        SmartDashboard.putNumber("Climber Amps", climberMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber RPM", climberMotor.getEncoder().getVelocity());
    }
}
