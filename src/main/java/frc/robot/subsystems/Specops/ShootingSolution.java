package frc.robot.subsystems.Specops;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.Specops;

public class ShootingSolution {
    public enum shooterState {kIdle, kTarget, kLow, kHigh}
    Shooter _shooter;

    PIDController topPID, bottomPID;

    double pitch, topSpeed, bottomSpeed;
    Pose2d currentpose, aprilTagPose;

    public ShootingSolution(){
        _shooter = Shooter.getInstance();
        topPID = new PIDController(Specops.kPTOP, 0, 0);
        bottomPID = new PIDController(Specops.kPBOTTOM, 0, 0);
    }
    public void dataIn(Pose2d currentpose, Pose2d aprilTagPose){
        this.currentpose = currentpose;
        this.aprilTagPose = aprilTagPose;
    }
    public void setState(shooterState State){
        switch(State){
            case kIdle:
                _shooter.setState(50, 0, 0);
                break;
            case kTarget:

                break;
            case kLow:
                double topSpeed1 = topPID.calculate(_shooter.getTopRPM(),Specops.kHighTopRPM);
                double bottomSpeed1 = bottomPID.calculate(_shooter.getBottomRPM(),Specops.kHighBottomRPM);
                _shooter.setState(50, topSpeed1, bottomSpeed1);
                break;
            case kHigh:
                double topSpeed2 = topPID.calculate(_shooter.getTopRPM(),Specops.kHighTopRPM);
                double bottomSpeed2 = bottomPID.calculate(_shooter.getBottomRPM(),Specops.kHighBottomRPM);
                _shooter.setState(50, topSpeed2, bottomSpeed2);
                break;
        }
    }
}
