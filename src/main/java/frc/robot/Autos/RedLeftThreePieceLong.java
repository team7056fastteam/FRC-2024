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

public class RedLeftThreePieceLong extends FastAutoBase{
    double[][] point0 = {{-0.18,35.94,48.31,3},{0,89.7,0,3},{-0.18,35.94,48.31,3}};
    double[][] point1 = {{-122.88, 162.39, 0, 12, 1},{-122.88, 298.14, 0, 12, 1},{-122.88, 306.14, 0, 3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path0),
            new FastSeries(List.of(
                new PassXYCommand(-0.18, 36, 4),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new PassXYCommand(0,70, 14),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-0.18, 36, 4),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new KurtinatorCommand(KurtinatorState.kIdle)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(-130,162, 14),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
    }
}
