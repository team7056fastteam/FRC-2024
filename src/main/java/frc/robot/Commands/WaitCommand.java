package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Autos.Common.FastCommand;

public class WaitCommand extends FastCommand{
    Timer time = new Timer();
    double waitTime;

    public WaitCommand(double seconds){
        waitTime = seconds;
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
        if(time.get() > waitTime){ return true;} else { return false;}
    }

    @Override
    public void end() {}
    
}
