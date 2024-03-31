package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Specops;

public class Pivotinator extends SubsystemBase{
    static final double maxCounts = 10727;
    double maxAngle = 49;
    double minAngle = 30;

    double targetHeight = 84;
    double dist = 84;

    public enum pivotState{kIdle, kPivoting, kHoming, kAutoAim}
    pivotState state = pivotState.kIdle;
    double angleSetPoint = 42.5;

    CANSparkMax pivotMotor;
    RelativeEncoder pivotEncoder;

    PIDController kPositionPid;

    DigitalInput UpperLimit = new DigitalInput(1);
    DigitalInput LowerLimit = new DigitalInput(2);

    public Pivotinator(){
        pivotMotor = new CANSparkMax(Specops.kPivotMotor, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        kPositionPid = new PIDController(Specops.kPPivot, Specops.kIPivot, 0);
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(pivotEncoder.getCountsPerRevolution());
    }
    public void setState(pivotState state){
        this.state = state;
    }
    public void run(){
        switch (this.state) {
            case kIdle:
              pivotSpeed(0);
              break;
            case kPivoting:
              double helpfulPower0;
              helpfulPower0 = kPositionPid.calculate(angleSetPoint, countsToAngle(pivotEncoder.getPosition()));
              helpfulPower0 = !UpperLimit.get() && helpfulPower0 < 0 ? 0 : helpfulPower0;
              helpfulPower0 = !LowerLimit.get() && helpfulPower0 > 0 ? 0 : helpfulPower0/9;
              SmartDashboard.putNumber("Pivot Power", helpfulPower0);
              pivotSpeed(helpfulPower0);
              break;
            case kAutoAim:
              double helpfulPower1;
              double pitch = pitchClamped(Math.toDegrees(Math.atan(targetHeight/dist)));
              helpfulPower1 = kPositionPid.calculate(pitch, countsToAngle(pivotEncoder.getPosition()));
              helpfulPower1 = !UpperLimit.get() && helpfulPower1 < 0 ? 0 : helpfulPower1;
              helpfulPower1 = !LowerLimit.get() && helpfulPower1 > 0 ? 0 : helpfulPower1/9;
              pivotSpeed(helpfulPower1);
              break;
            case kHoming:
              if(!UpperLimit.get()){
                pivotSpeed(0);
                state = pivotState.kPivoting;
                pivotEncoder.setPosition(0);
              }
              else{
                pivotSpeed(-0.15);
              }
              break;
        }       
    }
    public void setPivotAngle(double angle){
        angleSetPoint = angle;
    }
    double pitchClamped(double pitch){
        return Math.max(minAngle, Math.min(maxAngle, pitch));
    }
    double countsToAngle(double counts){
        return pitchClamped(maxAngle - ((Math.abs(counts/maxCounts)) * (maxAngle - minAngle)));
      }
    void pivotSpeed(double speed){
        pivotMotor.set(speed);
    }
    public void Dashboard(){
        SmartDashboard.putString("Pivot State", state.toString());
        SmartDashboard.putBoolean("Top Tripped", !UpperLimit.get());
        SmartDashboard.putBoolean("Bottom Tripped", !LowerLimit.get());
        SmartDashboard.putNumber("Pivot Angle", countsToAngle(pivotEncoder.getPosition()));
        SmartDashboard.putNumber("target Pivot Angle", angleSetPoint);
    }
}
