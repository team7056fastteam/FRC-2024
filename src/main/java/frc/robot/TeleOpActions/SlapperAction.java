package frc.robot.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Common.FastTeleOpAction;
import frc.robot.subsystems.Specops.Slapper.SlappState;

public class SlapperAction extends FastTeleOpAction{
    public SlappState state;

    public SlapperAction(SlappState state){
        this.state = state;
    }

    @Override
    public void run() {
        Robot._slapper.setState(state);
    }
    
}
