package frc.robot.Autos;

import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.FastParallel;
import frc.robot.Autos.Common.FastSeries;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueLeftThreePiece extends FastAutoBase{
    //{x,y,heading,error}
    double[][] path0 = {{-12.53,28.80,65,2}};
    double[][] path1 = {{-1.56,73.71,0,3}};
    double[][] path2 = {{54.59,58.58,0,3}};
    double[][] path3 = {{54.24,78.41,0,3}};

    double[][][] paths = {path0, path1, path2, path3};

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(paths, 0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));

        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new RunPathCommand(paths,1));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0.5, 0, 0), 
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward), 
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new WaitForIntake(4))))));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0, 0, 0),
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip))))));
        
        runCommand(new RunPathCommand(paths,2));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new RunPathCommand(paths,3));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0.5, 0, 0), 
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward), 
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new WaitForIntake(4))))));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0, 0, 0),
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip))))));
        runCommand(new IngestCommand(IngestState.kIdle));
        runCommand(new RunPathCommand(paths,2));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
}
