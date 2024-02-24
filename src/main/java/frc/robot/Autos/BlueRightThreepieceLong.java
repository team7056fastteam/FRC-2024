package frc.robot.Autos;
// import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
// import frc.robot.Autos.Common.FastParallel;
// import frc.robot.Autos.Common.FastSeries;
import frc.robot.Autos.Common.Path;
import frc.robot.Autos.Common.Path.WayPointBehavior;
import frc.robot.Commands.*;
// import frc.robot.subsystems.Specops.Ingest.IngestState;
// import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
// import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightThreepieceLong extends FastAutoBase{
    double[][] point0 = {{-5.58,40.21,314.69,3}, {-2.85,80.67,0,3.5}, {-5.58,40.21,314.69,5}};
    double[][] point1 = {{60.83, 65.49,0,3},{122.61, 221.11,0,3},{126.73,300.49,0,3},{115.38, 92.38,0,6}};
    double[][] point2 = {{-5.58,40.21,314.69,3}};
    double[][] point3 = {{0,100,0,14,2}, {-5,100,0,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path2 = new Path(point2, WayPointBehavior.Standard);
    Path path3 = new Path(point3, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        // runCommand(new ShooterCommand(shooterState.kHigh));
        // runCommand(new FastParallel(List.of(
        //     new RunPathCommand(paths, 0, Speed.Slow),
        //     new FastSeries(List.of(
        //         new PassXYCommand(-5.5, 40.21, 5),
        //         new KurtinatorCommand(KurtinatorState.kFeed),
        //         new IngestCommand(IngestState.kIdle),
        //         new PassXYCommand(-2.85, 80.5, 7),
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip),
        //         new IngestCommand(IngestState.kForward),
        //         new PassXYCommand(-5.5, 40.21, 3),
        //         new KurtinatorCommand(KurtinatorState.kFeed),
        //         new IngestCommand(IngestState.kIdle)
        // )))));
        // runCommand(new FastParallel(List.of(
        //     new RunPathCommand(paths, 1, Speed.Fast),
        //     new FastSeries(List.of(
        //         new PassXYCommand(122, 221, 8),
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip),
        //         new IngestCommand(IngestState.kForward)
        // )))));
        // runCommand(new FastParallel(List.of(
        //     new RunPathCommand(paths, 2, Speed.Slow),
        //     new FastSeries(List.of(
        //         new PassXYCommand(-5.5, 40.21, 3),
        //         new KurtinatorCommand(KurtinatorState.kFeed),
        //         new IngestCommand(IngestState.kIdle)
        // )))));
        runCommand(new RunPathCommand(path3));
        runCommand(new WaitCommand(0.5));
        runCommand(new StopCommand());
    }
}
