package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.subsystems.Specops.Kurtinator;

public class WaitForIntake extends FastCommand{
    Timer time = new Timer();
    double maxWait;

    Kurtinator _kurt = Robot.getKurtinatorInstance();

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
        return (maxWait < time.get()) || _kurt.LimitSwitchTripped();
    }

    @Override
    public void end() {
        time.stop();
    }
    
}
