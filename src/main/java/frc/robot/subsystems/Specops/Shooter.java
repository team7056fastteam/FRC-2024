package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;
import frc.robot.Constants.Specops;

public class Shooter extends SubsystemBase{
    ShootingSolution _solution = new ShootingSolution(this);

    double pitch, topSpeed, bottomSpeed;

    CANSparkMax ShooterMotorTop;
    CANSparkMax ShooterMotorBottom;

    RelativeEncoder TopEncoder;
    RelativeEncoder BottomEncoder;

    public Shooter(){
        ShooterMotorTop = new CANSparkMax(Specops.kShooterMotorBottom, MotorType.kBrushless);
        ShooterMotorBottom = new CANSparkMax(Specops.kShooterMotorTop, MotorType.kBrushless);

        ShooterMotorTop.setIdleMode(IdleMode.kCoast);
        ShooterMotorBottom.setIdleMode(IdleMode.kCoast);

        TopEncoder = ShooterMotorTop.getEncoder();
        BottomEncoder = ShooterMotorBottom.getEncoder();

        TopEncoder.setVelocityConversionFactor(1);
        BottomEncoder.setVelocityConversionFactor(1);

        pitch = 50;
    }

    public void setState(double pitch, double topSpeed, double bottomSpeed){
        this.pitch = pitch;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    public void run(){
        setTopSpeed(topSpeed);
        setBottomSpeed(bottomSpeed);
        _solution.run();
    }

    public void setSolutionState(shooterState State){
        _solution.setState(State);
    }

    public double getTopRPM(){
        return TopEncoder.getVelocity();
    }
    public double getBottomRPM(){
        return BottomEncoder.getVelocity();
    }

    void setTopSpeed(double speed){
        ShooterMotorTop.set(speed);
    }
    void setBottomSpeed(double speed){
        ShooterMotorBottom.set(speed);
    }

    public void Dashboard(){
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Top", TopEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom", BottomEncoder.getVelocity());
        SmartDashboard.putString("Shooter Sate", _solution.State.toString());
    }
}
