package frc.robot.AutoCommands;

import frc.robot.Robot;
import frc.robot.Common.FastCommand;

public class PassXYCommand extends FastCommand{
    double targetx, targety, targetError;

    public PassXYCommand(double x, double y, double error){
        targetx = x;
        targety = y;
        targetError = error;
    }

    @Override
    public void init() {}

    @Override
    public void run() {}

    @Override
    public Boolean isFinished() {
        return Math.abs(targetx - Robot.getPose().getX()) < targetError && Math.abs(targety - Robot.getPose().getY()) < targetError;
    }

    @Override
    public void end() {
        System.out.println("Successfully Passed Point with an Error of " + targetError);
    }
    
}
