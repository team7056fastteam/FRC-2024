package frc.robot.subsystems.Specops;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;

public class NoteState {
    public enum noteState{kNote, kNoNote}
    public noteState state = noteState.kNoNote;

    public void run(){
        if(!Robot._kurtinator.LimitSwitchTripped()){
            state = noteState.kNote;
        }
        if(Robot._kurtinator.state == KurtinatorState.kFeed || Robot._kurtinator.state == KurtinatorState.kReversed){
            state = noteState.kNoNote;
        }
    }

    public void Dashboard(){
        SmartDashboard.putString("Note State", state.toString());
    }
}
