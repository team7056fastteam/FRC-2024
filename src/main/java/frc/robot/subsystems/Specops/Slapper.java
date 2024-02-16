package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Specops;

public class Slapper {
    CANSparkMax slapperMotor;

    public Slapper(){
        slapperMotor = new CANSparkMax(Specops.kSlapperMotor, MotorType.kBrushless);
    }

    void slapperPower(double power){
        slapperMotor.set(power);
    }
}
