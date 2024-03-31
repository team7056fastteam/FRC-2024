package frc.robot.AutoCommands;

import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.subsystems.Specops.Pivotinator.pivotState;

public class PivotCommand extends FastCommand{
    pivotState state = pivotState.kIdle;
    double angle = 48;
    public PivotCommand(pivotState state, double angle){
        this.state = state;
        this.angle = angle;
    }

    @Override
    public void init() {
        Robot._pivot.setState(state);
        Robot._pivot.setPivotAngle(angle);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return true;
    }

    @Override
    public void end() {}
    
}
