package frc.robot.Autos;

import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.FastParallel;
import frc.robot.Autos.Common.FastSeries;
import frc.robot.Autos.Common.Path;
import frc.robot.Autos.Common.Path.WayPointBehavior;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightThreePiece  extends FastAutoBase{
    //{x,y,heading,error}
    double[][] point0 = {{-5.58,40.21,314.69,3}};
    double[][] point1 = {{-2.85,80.67,0,3.5},{-60.85, 59.38, 0, 3},{-61.86, 95.99,0,3},{-60.85,59.38,0,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(-2.85, 80.5, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-50, 59.38, 5),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle),
                new PassXYCommand(-60, 90, 5),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-60.85, 59.38, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
    
}
