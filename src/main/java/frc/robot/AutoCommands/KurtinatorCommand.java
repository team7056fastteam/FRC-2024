package frc.robot.AutoCommands;

import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;

public class KurtinatorCommand extends FastCommand{
    KurtinatorState state = KurtinatorState.kIdle;
    public KurtinatorCommand(KurtinatorState state){
        this.state = state;
    }

    @Override
    public void init() {
        Robot._kurtinator.setState(state);
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
