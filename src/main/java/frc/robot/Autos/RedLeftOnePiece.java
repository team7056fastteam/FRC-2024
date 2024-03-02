package frc.robot.Autos;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.Path;
import frc.robot.Autos.Common.Path.WayPointBehavior;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class RedLeftOnePiece extends FastAutoBase{
    double[][] point0 = {{-8.53,26.80,65,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(3));
        runCommand(new StopCommand());
    }
    
}
