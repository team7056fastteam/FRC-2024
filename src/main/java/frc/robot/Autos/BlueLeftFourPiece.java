package frc.robot.Autos;

import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.FastParallel;
import frc.robot.Autos.Common.FastSeries;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueLeftFourPiece extends FastAutoBase{
    //{x,y,heading,error} 
    double[][] path0 = {{-8.53,26.80,65,3}, {-4.56,73.71,0,3}};
    double[][] path1 = {{-4.56,85.71,0,3}, {50.59,58.58,0,3}, {52.76, 87.51,0,3}};
    double[][] path2 = {{89.68, 60.07,320.44,3},{106.47,89.29, 338.44, 3}, {89.68, 60.07,338.44,3}};

    double[][][] paths = {path0, path1, path2};

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(paths, 0),
            new FastSeries(List.of(
                new PassXYCommand(-8.5, 26.8, 3),
                new KurtinatorCommand(KurtinatorState.kFeed)
        )))));
        runCommand(new KurtinatorCommand(KurtinatorState.kRunTilTrip));
        runCommand(new IngestCommand(IngestState.kForward));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(paths, 1),
            new FastSeries(List.of(
                new PassXYCommand(50.59, 55.58, 10),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new PassXYCommand(54.59, 70.58, 5),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(paths, 2),
            new FastSeries(List.of(
                new PassXYCommand(87.68, 60.07, 10),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new PassXYCommand(90.47,80.29, 12),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        // runCommand(new RunPathCommand(paths,1));
        // runCommand(new FastParallel(List.of(
        //     new DriveCommand(0.5, 0, 0), 
        //     new FastSeries(List.of(
        //         new IngestCommand(IngestState.kForward), 
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip),
        //         new WaitForIntake(4))))));
        // runCommand(new FastParallel(List.of(
        //     new DriveCommand(0, 0, 0),
        //     new FastSeries(List.of(
        //         new IngestCommand(IngestState.kForward),
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip))))));
        
        // runCommand(new RunPathCommand(paths,2));
        // runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        // runCommand(new WaitCommand(1));
        // runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        // runCommand(new RunPathCommand(paths,3));
        // runCommand(new FastParallel(List.of(
        //     new DriveCommand(0.5, 0, 0), 
        //     new FastSeries(List.of(
        //         new IngestCommand(IngestState.kForward), 
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip),
        //         new WaitForIntake(4))))));
        // runCommand(new FastParallel(List.of(
        //     new DriveCommand(0, 0, 0),
        //     new FastSeries(List.of(
        //         new IngestCommand(IngestState.kForward),
        //         new KurtinatorCommand(KurtinatorState.kRunTilTrip))))));
        // runCommand(new IngestCommand(IngestState.kIdle));
        // runCommand(new RunPathCommand(paths,2));
        // runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        // runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
}
