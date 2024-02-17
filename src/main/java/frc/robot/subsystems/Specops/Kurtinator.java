package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;

public class Kurtinator extends SubsystemBase{
    CANSparkMax KurtinatorMotor;

    DigitalInput left;

    public enum KurtinatorState{kFeed, kRunTilTrip, kReversed, kIdle}
    KurtinatorState state = KurtinatorState.kIdle;

    public Kurtinator(){
        left = new DigitalInput(Specops.kLeftLimit);
        KurtinatorMotor = new CANSparkMax(Specops.kKurtinatorMotor, MotorType.kBrushless);
    }
    public void setState(KurtinatorState state){
        this.state = state;

        run();
    }
    public void run(){
        switch(this.state){
            case kFeed:
                setKurtinatorSpeed(Specops.kKurtinatorForwardSpeed);
                break;
            case kRunTilTrip:
                if(LimitSwitchTripped()){ setKurtinatorSpeed(Specops.kKurtinatorForwardSpeed); } else { setKurtinatorSpeed(0); }
                break;
            case kReversed:
                setKurtinatorSpeed(Specops.kKurtinatorReversedSpeed);
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
        return !left.get();
    }
    public void Dashboard(){
        SmartDashboard.putString("Kurtinator State", state.toString());
        SmartDashboard.putBoolean("Tripped", LimitSwitchTripped());
    }
}
