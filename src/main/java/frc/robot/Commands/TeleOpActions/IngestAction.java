package frc.robot.Commands.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastTeleOpAction;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;

public class IngestAction extends FastTeleOpAction{
    IngestState ingestState = IngestState.kIdle;
    KurtinatorState kurtinatorState = KurtinatorState.kIdle;

    public IngestAction(IngestState ingestState, KurtinatorState kurtinatorState){
        this.ingestState = ingestState;
        this.kurtinatorState = kurtinatorState;
    }

    @Override
    public void run() {
        if(ingestState == IngestState.kForward){
            if(Robot._kurtinator.LimitSwitchTripped()){
                Robot._ingest.setState(IngestState.kIdle);
                Robot._kurtinator.setState(KurtinatorState.kIdle);
            }
            else{
                Robot._ingest.setState(ingestState);
                Robot._kurtinator.setState(kurtinatorState);
            }
        }
        else{
            Robot._ingest.setState(ingestState);
            Robot._kurtinator.setState(kurtinatorState);
        }
    }
    
}
