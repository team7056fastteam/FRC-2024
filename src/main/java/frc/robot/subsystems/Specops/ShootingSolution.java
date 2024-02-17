package frc.robot.subsystems.Specops;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Specops;
//import frc.robot.subsystems.Specops.Pitchinator.PitchState;

public class ShootingSolution {
    public enum shooterState {kIdle, kTarget, kLow, kHigh}
    private Shooter _shooter;
    //private Pitchinator _pitch = new Pitchinator();

    public shooterState State = shooterState.kIdle;

    double robotHeight = 0;
    double targetHeight = 7*12;
    double tx = 0;
    double ta = 0;

    double topSpeed0, topSpeed1, topSpeed2;
    double bottomSpeed0, bottomSpeed1, bottomSpeed2;

    PIDController topPID, bottomPID;

    Pose2d currentpose;

    public ShootingSolution(Shooter _shooter){
        topPID = new PIDController(Specops.kPTOP, 0, 0);
        bottomPID = new PIDController(Specops.kPBOTTOM, 0, 0);
        this._shooter = _shooter;
    }
    public void dataIn(Pose2d currentpose, double tx, double ta){
        this.currentpose = currentpose;
        this.tx = tx;
        this.ta = ta;
    }
    public void setState(shooterState State){
        this.State = State;
    }

    public void run(){
        switch(this.State){
            case kIdle:
                _shooter.setState(50, 0, 0);
                bottomPID.reset();
                topPID.reset();
                //_pitch.setState(PitchState.kPitching);
                break;
            case kTarget:
                double dist = 15.552/ta;
                double pitch = pitchClamped(Math.toDegrees(Math.atan(targetHeight/dist)));
                double yaw = tx;
                topSpeed0 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kHighTopRPM);
                bottomSpeed0 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kHighBottomRPM);
                _shooter.setState(pitch, topSpeed0, bottomSpeed0);
                //_pitch.setState(PitchState.kPitching);
                _shooter.setYaw(yaw);
                break;
            case kLow:
                topSpeed1 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kLowTopRPM);
                bottomSpeed1 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kLowBottomRPM);
                _shooter.setState(50, topSpeed1, bottomSpeed1);
                //_pitch.setState(PitchState.kPitching);
                break;
            case kHigh:
                topSpeed2 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kHighTopRPM);
                bottomSpeed2 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kHighBottomRPM);
                _shooter.setState(50, topSpeed2, bottomSpeed2);
                //_pitch.setState(PitchState.kPitching);
                break;
        }
    }

    double pitchClamped(double pitch){
        return Math.max(20, Math.min(52, pitch));
    }
}
