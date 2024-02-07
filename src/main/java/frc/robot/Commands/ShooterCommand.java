package frc.robot.Commands;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class ShooterCommand extends FastCommand{
    shooterState state = shooterState.kIdle;
    public ShooterCommand(shooterState state){
        this.state = state;
    }

    @Override
    public void init() {
        Robot._shooter.setSolutionState(state);
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
