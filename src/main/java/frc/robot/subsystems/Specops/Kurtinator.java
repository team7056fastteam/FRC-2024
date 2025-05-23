package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;
import frc.robot.subsystems.Specops.NoteState.noteState;
import frc.robot.Robot;

public class Kurtinator extends SubsystemBase{
    CANSparkMax KurtinatorMotor;

    DigitalInput left,noteSensor;

    public enum KurtinatorState{kFeed, kRunTilTrip, kReversed, kIdle, kFeedFast}
    KurtinatorState state = KurtinatorState.kIdle;

    public Kurtinator(){
        left = new DigitalInput(Specops.kLeftLimit);
        noteSensor = new DigitalInput(3);
        KurtinatorMotor = new CANSparkMax(Specops.kKurtinatorMotor, MotorType.kBrushless);
    }
    public void setState(KurtinatorState state){
        this.state = state;
        if(DriverStation.isAutonomous()){
            System.out.println("Kurtinator State Changed to " + state.toString());
        }
    }
    public void run(){
        switch(this.state){
            case kFeed:
                setKurtinatorSpeed(Specops.kKurtinatorForwardSpeed);
                break;
            case kRunTilTrip:
                if(Robot._noteState.state == noteState.kNoNote){ setKurtinatorSpeed(Specops.kKurtinatorExperimentalForwardSpeed); } else { setKurtinatorSpeed(0); }
                break;
            case kReversed:
                setKurtinatorSpeed(Specops.kKurtinatorReversedSpeed);
                break;
            case kFeedFast:
                setKurtinatorSpeed(Specops.kKurtinatorExperimentalForwardSpeed);
                break;
            case kIdle:
                setKurtinatorSpeed(0);
                break;
        }
    }

    void setKurtinatorSpeed(double speed){
        KurtinatorMotor.set(speed);
    }

    public boolean LimitSwitchTripped(){
        //return !left.get() || noteSensor.get();
        return noteSensor.get();
    }
    public void Dashboard(){
        SmartDashboard.putString("Kurtinator State", state.toString());
        SmartDashboard.putBoolean("Tripped", !LimitSwitchTripped());
    }
}
