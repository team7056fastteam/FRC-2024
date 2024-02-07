package frc.robot.Commands.TeleOpActions;

import frc.robot.Robot;
import frc.robot.Autos.Common.FastTeleOpAction;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class ShooterAction extends FastTeleOpAction{
    shooterState state = shooterState.kIdle;
    public ShooterAction(shooterState state){
        this.state = state;
    }

    @Override
    public void run() {
        Robot._shooter.setSolutionState(state);
    }
}
