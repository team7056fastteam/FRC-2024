package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;

public class Ingest extends SubsystemBase{
    public enum IngestState{ kIdle, kForward, kReversed}
    IngestState state = IngestState.kIdle;

    CANSparkMax IngestMotor;

    public Ingest(){
        IngestMotor = new CANSparkMax(Specops.kIngestMotor, MotorType.kBrushless);
    }

    public void setState(IngestState state){
        this.state = state;
        switch(state){
            case kIdle:
                setIngestSpeed(0);
                break;
            case kForward:
                setIngestSpeed(Specops.kIngestForwardSpeed);
                break;
            case kReversed:
                setIngestSpeed(Specops.kIngestReversedSpeed);
                break;
        }
    }

    void setIngestSpeed(double speed){
        IngestMotor.set(speed);
    }

    public void Dashboard(){
        SmartDashboard.putString("Ingest State", state.toString());
    }
}
