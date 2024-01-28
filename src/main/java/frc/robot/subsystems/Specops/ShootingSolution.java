package frc.robot.subsystems.Specops;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Constants.Specops;
import frc.robot.subsystems.Specops.Pitchinator.PitchState;

public class ShootingSolution {
    public enum shooterState {kIdle, kTarget, kLow, kHigh}
    private Shooter _shooter = Robot.getShooterInstance();
    private Pitchinator _pitch = new Pitchinator();

    public shooterState State = shooterState.kIdle;

    double robotHeight = 0;
    double targetHeight = 7*12;
    double tx = 0;
    double ta = 0;

    PIDController topPID, bottomPID;

    Pose2d currentpose;

    public ShootingSolution(){
        topPID = new PIDController(Specops.kPTOP, 0, 0);
        bottomPID = new PIDController(Specops.kPBOTTOM, 0, 0);
    }
    public void dataIn(Pose2d currentpose, double tx, double ta){
        this.currentpose = currentpose;
        this.tx = tx;
        this.ta = ta;
    }
    public void setState(shooterState State){
        this.State = State;
        switch(this.State){
            case kIdle:
                _shooter.setState(50, 0, 0);
                _pitch.setState(PitchState.kPitching);
                break;
            case kTarget:
                double dist = 15.552/ta;
                double pitch = pitchClamped(Math.toDegrees(Math.atan(targetHeight/dist)));
                double yaw = tx;
                double topSpeed0 = topPID.calculate(_shooter.getTopRPM(),Specops.kHighTopRPM);
                double bottomSpeed0 = bottomPID.calculate(_shooter.getBottomRPM(),Specops.kHighBottomRPM);
                _shooter.setState(pitch, topSpeed0, bottomSpeed0);
                _pitch.setState(PitchState.kPitching);
                _shooter.setYaw(yaw);
                break;
            case kLow:
                double topSpeed1 = topPID.calculate(_shooter.getTopRPM(),Specops.kHighTopRPM);
                double bottomSpeed1 = bottomPID.calculate(_shooter.getBottomRPM(),Specops.kHighBottomRPM);
                _shooter.setState(50, topSpeed1, bottomSpeed1);
                _pitch.setState(PitchState.kPitching);
                break;
            case kHigh:
                double topSpeed2 = topPID.calculate(_shooter.getTopRPM(),Specops.kHighTopRPM);
                double bottomSpeed2 = bottomPID.calculate(_shooter.getBottomRPM(),Specops.kHighBottomRPM);
                _shooter.setState(50, topSpeed2, bottomSpeed2);
                _pitch.setState(PitchState.kPitching);
                break;
        }
    }

    double pitchClamped(double pitch){
        return Math.max(20, Math.min(52, pitch));
    }
}
