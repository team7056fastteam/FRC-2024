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
    double[][] path0 = {{-11.39,28.10,61.81,2}};
    double[][] path1 = {{-1.56,73.71,0,2}};
    double[][] path2 = {{54.59,56.58,0,3}};
    double[][] path3 = {{48.34,73.62,0,3}};

    double[][][] paths = {path0, path1, path2, path3};

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));

        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new RunPathCommand(path1));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0.2, 0, 0), 
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward), 
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new WaitForIntake(2))))));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0, 0, 0),
            new FastSeries(List.of(
                new IngestCommand(IngestState.kIdle),
                new KurtinatorCommand(KurtinatorState.kIdle))))));
        
        runCommand(new RunPathCommand(path2));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));

        runCommand(new RunPathCommand(path3));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0.2, 0, 0), 
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward), 
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new WaitForIntake(2))))));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0, 0, 0),
            new FastSeries(List.of(
                new IngestCommand(IngestState.kIdle),
                new KurtinatorCommand(KurtinatorState.kIdle))))));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
}
