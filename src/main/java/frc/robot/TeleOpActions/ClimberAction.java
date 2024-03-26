package frc.robot.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Common.FastTeleOpAction;
import frc.robot.subsystems.Specops.Climber.ClimbState;

public class ClimberAction extends FastTeleOpAction{
    ClimbState state = ClimbState.kIdle;
    public ClimberAction(ClimbState state){
        this.state = state;
    }

    @Override
    public void run() {
        Robot._climber.setState(state);
    }
}
