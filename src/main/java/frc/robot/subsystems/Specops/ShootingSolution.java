package frc.robot.subsystems.Specops;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Specops;
//import frc.robot.subsystems.Specops.Pitchinator.PitchState;

public class ShootingSolution {
    public enum shooterState {kIdle, kTarget, kLow, kHigh, kPass}
    private Shooter _shooter;

    public shooterState State = shooterState.kIdle;

    double yaw = 0;

    double topSpeed0, topSpeed1, topSpeed2, topSpeed3;
    double bottomSpeed0, bottomSpeed1, bottomSpeed2, bottomSpeed3;

    PIDController topPID, bottomPID;
    Translation2d blueGoal = new Translation2d(0,60);

    public ShootingSolution(Shooter _shooter){
        topPID = new PIDController(Specops.kPTOP, 0.000000001, 0);
        bottomPID = new PIDController(Specops.kPBOTTOM, 0.000000001, 0);
        this._shooter = _shooter;
    }
    public void setState(shooterState State){
        this.State = State;
        if(DriverStation.isAutonomous()){
            System.out.println("Shooter State Changed to " + State.toString());
        }
    }

    public void run(){
        switch(this.State){
            case kIdle:
                _shooter.setState(0, 0);
                bottomPID.reset();
                topPID.reset();
                topSpeed3 = 0;
                bottomSpeed3 = 0;
                topSpeed2 = 0;
                bottomSpeed2 = 0;
                topSpeed1 = 0;
                bottomSpeed1 = 0;
                topSpeed0 = 0;
                bottomSpeed0 = 0;
                break;
            case kTarget:
                topPID.setP(Specops.kPTOP);
                bottomPID.setP(Specops.kPBOTTOM);
                topSpeed0 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kHighTopRPM);
                bottomSpeed0 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kHighBottomRPM);
                _shooter.setState(topSpeed0, bottomSpeed0);
                break;
            case kLow:
                topPID.setP(Specops.kAmpPTOP);
                bottomPID.setP(Specops.kAmpPBOTTOM);
                topSpeed1 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kLowTopRPM);
                bottomSpeed1 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kLowBottomRPM);
                _shooter.setState(topSpeed1, bottomSpeed1);
                break;
            case kHigh:
                topPID.setP(Specops.kPTOP);
                bottomPID.setP(Specops.kPBOTTOM);
                topSpeed2 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kHighTopRPM);
                bottomSpeed2 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kHighBottomRPM);
                _shooter.setState(topSpeed2, bottomSpeed2);
                break;
            case kPass:
                topPID.setP(Specops.kPTOP);
                bottomPID.setP(Specops.kPBOTTOM);
                topSpeed3 += topPID.calculate(Math.abs(_shooter.getTopRPM()),Specops.kMidTopRPM);
                bottomSpeed3 += bottomPID.calculate(Math.abs(_shooter.getBottomRPM()),Specops.kMidBottomRPM);
                _shooter.setState(topSpeed3, bottomSpeed3);
                break;
        }
    }
}
