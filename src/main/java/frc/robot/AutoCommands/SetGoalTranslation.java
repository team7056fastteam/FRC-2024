package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;

public class SetGoalTranslation extends FastCommand{
    Translation2d xY;

    public SetGoalTranslation(Translation2d xY){
        this.xY = xY;
    }

    @Override
    public void init() {
        Robot.setGoalTranslation(xY);
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
