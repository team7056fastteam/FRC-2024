package frc.robot.Autos;

import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.FastParallel;
import frc.robot.Autos.Common.FastSeries;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightTwoPiece  extends FastAutoBase{
    //{x,y,heading,error}
    double[][] path0 = {{-5.58,40.21,314.69,3}};
    double[][] path1 = {{-2.85,80.67,0,3.5}};

    double[][][] paths = {path0, path1};

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(paths, 0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));

        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new RunPathCommand(paths,1));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0.15, 0, 0), 
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward), 
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new WaitForIntake(0.75))))));
        runCommand(new FastParallel(List.of(
            new DriveCommand(0, 0, 0),
            new FastSeries(List.of(
                new IngestCommand(IngestState.kForward),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip))))));
        
        runCommand(new RunPathCommand(paths,0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
    
}
