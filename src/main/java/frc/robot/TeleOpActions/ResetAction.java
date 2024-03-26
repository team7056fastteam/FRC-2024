package frc.robot.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Common.FastTeleOpAction;

public class ResetAction extends FastTeleOpAction{
    
    @Override
    public void run() {
        Robot.setXY(0,0);
        Robot.resetH();
    }
}
