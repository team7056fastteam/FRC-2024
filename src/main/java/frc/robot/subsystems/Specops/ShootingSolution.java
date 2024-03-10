package frc.robot.subsystems.Specops;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.Constants.Specops;
//import frc.robot.subsystems.Specops.Pitchinator.PitchState;

public class ShootingSolution {
    public enum shooterState {kIdle, kTarget, kLow, kHigh}
    private Shooter _shooter;
    //private Pitchinator _pitch = new Pitchinator();

    public shooterState State = shooterState.kIdle;

    double robotHeight = 0;
    double targetHeight = 7*12;

    double dist = 0;
    double yaw = 0;

    double topSpeed0, topSpeed1, topSpeed2;
    double bottomSpeed0, bottomSpeed1, bottomSpeed2;

    PIDController topPID, bottomPID;
    Translation2d blueGoal = new Translation2d(0,-57);
    Translation2d redGoal = new Translation2d(0,0);

    public ShootingSolution(Shooter _shooter){
        topPID = new PIDController(Specops.kPTOP, 0.000000001, 0);
        bottomPID = new PIDController(Specops.kPBOTTOM, 0.000000001, 0);
        this._shooter = _shooter;
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
                topSpeed2 = 0;
                bottomSpeed2 = 0;
                topSpeed1 = 0;
                bottomSpeed1 = 0;
                topSpeed0 = 0;
                bottomSpeed0 = 0;
                //_pitch.setState(PitchState.kPitching);
                break;
            case kTarget:
                // Robot.updateNavPodWithVision();
                // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                //     dist = Math.sqrt(Math.pow((blueGoal.getY() - Robot.getPose().getY()), 2) + Math.pow((blueGoal.getX() - Robot.getPose().getX()), 2));
                //     yaw = Math.toDegrees(Math.atan2((blueGoal.getY() - Robot.getPose().getX()), (Robot.getPose().getY())));
                //     yaw = yaw < 0 ? yaw + 360 : yaw;

                // }
                // else{
                //     dist = Math.sqrt(Math.pow((blueGoal.getY() - Robot.getPose().getY()), 2) + Math.pow((blueGoal.getX() - Robot.getPose().getX()), 2));
                //     yaw = Math.toDegrees(Math.atan2((redGoal.getY() - Robot.getPose().getY()), (redGoal.getX() - Robot.getPose().getX())));
                //     yaw = yaw < 0 ? yaw + 360 : yaw;
                // }
                topPID.setP(Specops.kPTOP);
                bottomPID.setP(Specops.kPBOTTOM);
                // dist = Robot.GetTA()/15.22;
                // yaw = Robot.GetTX();
                double pitch = pitchClamped(Math.toDegrees(Math.atan(targetHeight/dist)));
                topSpeed0 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kHighTopRPM);
                bottomSpeed0 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kHighBottomRPM);
                _shooter.setState(pitch, topSpeed0, bottomSpeed0);
                //_pitch.setState(PitchState.kPitching);
                _shooter.setYaw(yaw);
                break;
            case kLow:
                topPID.setP(Specops.kAmpPTOP);
                bottomPID.setP(Specops.kAmpPBOTTOM);
                topSpeed1 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kLowTopRPM);
                bottomSpeed1 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kLowBottomRPM);
                _shooter.setState(50, topSpeed1, bottomSpeed1);
                //_pitch.setState(PitchState.kPitching);
                break;
            case kHigh:
                topPID.setP(Specops.kPTOP);
                bottomPID.setP(Specops.kPBOTTOM);
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
