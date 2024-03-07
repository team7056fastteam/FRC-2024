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

public class BlueLeftFourPiece extends FastAutoBase{
    //{x,y,heading,error} 
    // double[][] point0 = {{-8.53,26.80,65,3}, {-4.56,73.71,0,3}};
    // double[][] point1 = {{-4.56,85.71,0,3}, {50.59,58.58,0,3}, {52.76, 87.51,0,3}};
    // double[][] point2 = {{89.68, 60.07,320.44,3},{106.47,89.29, 338.44, 3}, {89.68, 60.07,338.44,3}};

    // Path path0 = new Path(point0, WayPointBehavior.Standard);
    // Path path1 = new Path(point1, WayPointBehavior.Standard);
    // Path path2 = new Path(point2, WayPointBehavior.Standard);

    // @Override
    // public void routine() throws Exception {
    //     runCommand(new ShooterCommand(shooterState.kHigh));
    //     runCommand(new FastParallel(List.of(
    //         new RunPathCommand(path0),
    //         new FastSeries(List.of(
    //             new PassXYCommand(-8.5, 26.8, 3),
    //             new KurtinatorCommand(KurtinatorState.kFeed)
    //     )))));
    //     runCommand(new KurtinatorCommand(KurtinatorState.kRunTilTrip));
    //     runCommand(new IngestCommand(IngestState.kForward));
    //     runCommand(new FastParallel(List.of(
    //         new RunPathCommand(path1),
    //         new FastSeries(List.of(
    //             new PassXYCommand(50.59, 55.58, 10),
    //             new KurtinatorCommand(KurtinatorState.kFeed),
    //             new PassXYCommand(54.59, 70.58, 5),
    //             new KurtinatorCommand(KurtinatorState.kRunTilTrip),
    //             new IngestCommand(IngestState.kForward)
    //     )))));
    //     runCommand(new FastParallel(List.of(
    //         new RunPathCommand(path2),
    //         new FastSeries(List.of(
    //             new PassXYCommand(87.68, 60.07, 10),
    //             new KurtinatorCommand(KurtinatorState.kFeed),
    //             new PassXYCommand(90.47,80.29, 12),
    //             new KurtinatorCommand(KurtinatorState.kRunTilTrip),
    //             new IngestCommand(IngestState.kForward)
    //     )))));
    //     runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
    //     runCommand(new WaitCommand(1));
    //     runCommand(new StopCommand());
    // }
    double[][] point0 = {{-8.53,26.80,65,3}};
    double[][] point1 = {{-4.56,73.71,0,3}, {50.59,58.58,0,3}};
    double[][] point2 = {{54.59, 87.51,0,3},{89.68, 60.07,320.44,3}};
    double[][] point3 = {{106.47,89.29, 338.44, 3}, {89.68, 60.07,338.44,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path2 = new Path(point2, WayPointBehavior.Standard);
    Path path3 = new Path(point3, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(-4.56, 73.71, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(50.59, 58.58, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(54.59, 87.51, 12),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(89.68, 60.07, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(106.47,87.29, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(89.68, 60.07, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
}
