package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Common.FastCommand;
import frc.robot.subsystems.Specops.NoteState.noteState;

public class WaitForIntake extends FastCommand{
    Timer time = new Timer();
    double maxWait;

    public WaitForIntake(double maxWait){
        this.maxWait = maxWait;
    }

    @Override
    public void init() {
       time.reset();
       time.start();
    }

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return (time.get() > maxWait) || Robot._noteState.state == noteState.kNote;
    }

    @Override
    public void end() {
        time.stop();
    }
    
}
