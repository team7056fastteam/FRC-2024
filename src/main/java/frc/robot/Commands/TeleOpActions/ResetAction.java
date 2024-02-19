package frc.robot.Commands.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastTeleOpAction;

public class ResetAction extends FastTeleOpAction{
    
    @Override
    public void run() {
        Robot.resetXY();
        Robot.resetH();
    }
}
