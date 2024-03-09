package frc.robot.AutoCommands;

import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.subsystems.Specops.Ingest.IngestState;

public class IngestCommand extends FastCommand{
    IngestState state = IngestState.kIdle;
    public IngestCommand(IngestState state){
        this.state = state;
    }

    @Override
    public void init() {
        Robot._ingest.setState(state);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() { return true; }

    @Override
    public void end() {}
}
