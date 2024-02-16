package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Specops;

public class Slapper {
    CANSparkMax slapperMotor;

    public enum SlappState{kSlapp, kUnSlapp, kIdle}
    SlappState state = SlappState.kIdle;

    public Slapper(){
        slapperMotor = new CANSparkMax(Specops.kSlapperMotor, MotorType.kBrushless);
    }

    public void setState(SlappState state){
        this.state = state;

        switch (this.state) {
            case kSlapp:
                if(slapperMotor.getOutputCurrent() > Specops.kSlappAmpLimit){
                    slapperPower(Specops.kSlappReducedForwardSpeed);
                }
                else{
                    slapperPower(Specops.kSlappForwardSpeed);
                }
                break;
            case kUnSlapp:
                if(slapperMotor.getOutputCurrent() > Specops.kSlappAmpLimit){
                    slapperPower(0);
                }
                else{
                    slapperPower(Specops.kSlappReversedSpeed);
                }
                break;
            case kIdle:
                slapperPower(0);
                break;
        }
    }

    void slapperPower(double power){
        slapperMotor.set(power);
    }

    public void Dashboard(){
        SmartDashboard.putBoolean("Slapp Amp Tripped", slapperMotor.getOutputCurrent() > Specops.kSlappAmpLimit);
        SmartDashboard.putString("Slapp State", state.toString());
        SmartDashboard.putNumber("Slapp Amp", slapperMotor.getOutputCurrent());
    }
}
