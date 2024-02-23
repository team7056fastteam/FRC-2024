package frc.robot.Autos;
import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.FastParallel;
import frc.robot.Autos.Common.FastSeries;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightThreepieceLong extends FastAutoBase{
    double[][] path0 = {{-5.58,40.21,314.69,3}, {-2.85,80.67,0,3.5}, {-5.58,40.21,314.69,5}};
    double[][] path1 = {{60.83, 65.49,0,3},{122.61, 221.11,0,3},{126.73,300.49,0,3},{115.38, 92.38,0,6}};
    double[][] path2 = {{-5.58,40.21,314.69,3}};
    double[][] path3 = {{100,5,0,14}, {100,36,0,3}};
    double[][][] paths = {path0, path1, path2, path3};

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
        runCommand(new RunPathCommand(paths, 3));
        runCommand(new WaitCommand(0.5));
        runCommand(new StopCommand());
    }
}
