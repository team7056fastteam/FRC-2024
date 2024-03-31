package frc.robot.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Common.FastTeleOpAction;
import frc.robot.subsystems.Specops.Pivotinator.pivotState;

public class PivotAction extends FastTeleOpAction{
    pivotState state = pivotState.kIdle;
    double angle = 48;

    public PivotAction(pivotState state,double angle){
        this.state = state;
        this.angle = angle;
    }
    @Override
    public void run() {
        Robot._pivot.setState(state);
        Robot._pivot.setPivotAngle(angle);
    }
}
