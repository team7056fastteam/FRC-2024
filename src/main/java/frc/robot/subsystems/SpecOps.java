package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;

public class SpecOps extends SubsystemBase {
    CANSparkMax ingestMotor, shooterMotor;

    // CANCoder armCoder, grabberMotorCoder, wristMotorCoder, extendMotorCoder;
    // RelativeEncoder armMotorEncoder;

    //DigitalInput limitSwitchIn;

    // PIDController armPidController, grabberPidController, wristPidController;

    // double wristOffset = 0, wristOffset2 = 0;
    // double restExtendEncoder;
    // double armMotorSpeed;

    public SpecOps(){
        //Motors
        ingestMotor = new CANSparkMax(Specops.kIngestMotor, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(Specops.kShooterMotor, MotorType.kBrushless);
        //wristMotor = new CANSparkMax(Specops.kWristMotor, MotorType.kBrushless);
        //grabberMotor = new CANSparkMax(Specops.kGrabberMotor, MotorType.kBrushless);
        

        //Encoders
        // armCoder = new CANCoder(Specops.kArmCoder);
        // armMotorEncoder = armMotor.getEncoder();

        // armMotorEncoder.setPosition(0);

        // wristMotorCoder = new CANCoder(Specops.kWristMotorCoder);
        // extendMotorCoder = new CANCoder(Specops.kExtendMotorCoder);
        // grabberMotorCoder = new CANCoder(Specops.kGrabberMotorCoder);

        //Pid Controllers
        // armPidController = new PIDController(0.8, 0.25, 0.05);
        // wristPidController = new PIDController(0.5, 0, 0);

        // wristOffset = wristMotorCoder.getPosition();
        // wristOffset2 = wristMotorCoder.getAbsolutePosition();

        //Limit Switches
        //limitSwitchIn = new DigitalInput(Specops.kLimitSwitchIn);

        //restExtendEncoder = extendMotorCoder.getPosition();
    }

    public void ingestPower(double power){
        ingestMotor.set(power);
    }

    public void shooterPower(double power){
        shooterMotor.set(power);
    }
    // public void armMotorPosition(double angle){
    //     armMotorSpeed = armPidController.calculate((armCoder.getAbsolutePosition()/100), (angle/100));
    //     if(-armMotorSpeed < 0 && armMotorEncoder.getPosition() > -Specops.armSpoolLimit){
    //         //spooling in which means encoder is reading negative
    //         armMotorPower(-armMotorSpeed);
    //     }
    //     else if(-armMotorSpeed > 0 && armMotorEncoder.getPosition() < 10){
    //         //spooling out which means encoder is reading positive
    //         double clampedSpeed = Math.min(.15, -armMotorSpeed);
    //         armMotorPower(clampedSpeed);
    //     }
    //     else{ 
    //         armMotorPower(0);
    //     }
    //     SmartDashboard.putNumber("angle", angle);
    //     SmartDashboard.putNumber("ArmMotorEncoder", armMotorEncoder.getPosition());
    //     SmartDashboard.putNumber("ArmPos", armCoder.getAbsolutePosition());
    // }
    // public void armMotorPower(double power){
    //     armMotor.set(power);
    //     SmartDashboard.putNumber("Arm", power);
    // }
    // public void grabberMotorPosition(double angle){
    //         double grabberMotorSpeed = (grabberPidController.calculate(grabberMotorCoder.getAbsolutePosition(), angle))/180;
    //         grabberMotor.set(-grabberMotorSpeed);
    // }

    // public void grabberMotorPower(double power){
    //     SmartDashboard.putNumber("GrabberPos", grabberMotorCoder.getAbsolutePosition());
    //     grabberMotor.set(power);
    // }
    // public void wristMotorPosition(double angle){
    //         double wristMotorSpeed = (wristPidController.calculate(wristMotorCoder.getAbsolutePosition(),angle))/180;
    //         wristMotor.set(-wristMotorSpeed);
    // }
    // public void extendMotorPower(double speed){
    //     extendMotor.set(speed);
    // }
    // public Boolean getLimitSwitchIn(){
    //     if(limitSwitchIn.get()){
    //         restExtendEncoder = extendMotorCoder.getPosition();
    //     }
    //     return limitSwitchIn.get();
    // }
    // public Boolean extendMotorRotLimit(){
    //     SmartDashboard.putNumber("rot", extendMotorCoder.getPosition() -  restExtendEncoder);
    //     if((extendMotorCoder.getPosition() - restExtendEncoder) < Specops.kExtendMotorRotLimit){
    //         return true;
    //     } else {return false;}
    // }

    // public double grabberMotorCoderget(){
    //     return grabberMotorCoder.getAbsolutePosition();
    // }
    // public void resetExtenderEncoder(){
    //     armMotorEncoder.setPosition(0);
    // }

    // public Boolean extendMotorRotLimitAuto(){
    //     SmartDashboard.putNumber("rot", extendMotorCoder.getPosition() -  restExtendEncoder);
    //     if((extendMotorCoder.getPosition() - restExtendEncoder) < 1950){
    //         return true;
    //     } else {return false;}
    // }
}