package frc.robot.subsystems.Specops;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;
import frc.robot.Constants.Specops;

public class Shooter extends SubsystemBase{
    ShootingSolution _solution = new ShootingSolution(this);

    double pitch, yaw;

    CANSparkMax ShooterMotorTop;
    CANSparkMax ShooterMotorBottom;

    RelativeEncoder TopEncoder;
    RelativeEncoder BottomEncoder;

    public Shooter(){
        ShooterMotorTop = new CANSparkMax(Specops.kShooterMotorBottom, MotorType.kBrushless);
        ShooterMotorBottom = new CANSparkMax(Specops.kShooterMotorTop, MotorType.kBrushless);

        TopEncoder = ShooterMotorTop.getEncoder();
        BottomEncoder = ShooterMotorBottom.getEncoder();

        TopEncoder.setVelocityConversionFactor(1);
        BottomEncoder.setVelocityConversionFactor(1);

        pitch = 50;
        yaw = 0;
    }

    public void setState(double pitch, double topSpeed, double bottomSpeed){
        this.pitch = pitch;

        setTopSpeed(topSpeed);
        setBottomSpeed(bottomSpeed);
    }

    public void setYaw(double yaw){
        this.yaw = yaw;
    }

    public double getYaw(){
        return yaw;
    }

    public void setSolutionState(shooterState State){
        _solution.setState(State);
    }

    public void dataInSolution(Pose2d currentPose, double tx, double ta){
        _solution.dataIn(currentPose, tx, ta);
    }

    public double getTopRPM(){
        return TopEncoder.getVelocity();
        //return 2500;
    }
    public double getBottomRPM(){
        return BottomEncoder.getVelocity();
        //return 2500;
    }

    void setTopSpeed(double speed){
        ShooterMotorTop.set(speed);
    }
    void setBottomSpeed(double speed){
        ShooterMotorBottom.set(speed);
    }

    public void Dashboard(){
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Top", TopEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom", BottomEncoder.getVelocity());
        SmartDashboard.putString("Shooter Sate", _solution.State.toString());
    }
}
