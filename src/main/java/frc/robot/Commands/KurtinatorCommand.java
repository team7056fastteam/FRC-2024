package frc.robot.Commands;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.subsystems.Specops.Kurtinator;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;

public class KurtinatorCommand extends FastCommand{
    KurtinatorState state = KurtinatorState.kIdle;
    Kurtinator _kurt = Robot.getKurtinatorInstance();
    public KurtinatorCommand(KurtinatorState state){
        this.state = state;
    }

    @Override
    public void init() {
        _kurt.setState(state);
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        if(state == KurtinatorState.kRunTilTrip){
            if(_kurt.LimitSwitchTripped()){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return true;
        }
    }

    @Override
    public void end() {
        if(state == KurtinatorState.kRunTilTrip){
            _kurt.setState(KurtinatorState.kIdle);
        }
    }
    
}
