package frc.robot.subsystems.Specops;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class Shooter extends SubsystemBase{
    public static Shooter mInstance;
    ShootingSolution _solution = new ShootingSolution(this);

    double pitch, yaw, topSpeed, bottomSpeed;

    public Shooter(){
        pitch = 50;
        yaw = 0;
        topSpeed = 0;
        bottomSpeed = 0;
    }

    public void setState(double pitch, double topSpeed, double bottomSpeed){
        this.pitch = pitch;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
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
        return 2500;
    }
    public double getBottomRPM(){
        return 2500;
    }
    public void Dashboard(){
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putString("Shooter Sate", _solution.State.toString());
    }
}
