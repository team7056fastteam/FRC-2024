package frc.robot.subsystems.Specops;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class Shooter extends SubsystemBase{
    public static Shooter mInstance;
    ShootingSolution _solution = new ShootingSolution();

    public static Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }

    double pitch, topSpeed, bottomSpeed;

    public Shooter(){
        pitch = 50;
        topSpeed = 0;
        bottomSpeed = 0;
    }

    public void setState(double pitch, double topSpeed, double bottomSpeed){
        this.pitch = pitch;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    public void setSolutionState(shooterState State){
        _solution.setState(State);
    }

    public double getTopRPM(){
        return 2500;
    }
    public double getBottomRPM(){
        return 2500;
    }
}
