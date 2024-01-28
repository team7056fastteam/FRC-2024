package frc.robot.Commands;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.subsystems.Specops.Ingest;
import frc.robot.subsystems.Specops.Ingest.IngestState;

public class IngestCommand extends FastCommand{
    IngestState state = IngestState.kIdle;
    private Ingest _ingest = Robot.getIngestInstance();
    public IngestCommand(IngestState state){
        this.state = state;
    }

    @Override
    public void init() {
        _ingest.setState(state);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() { return true; }

    @Override
    public void end() {}
}
