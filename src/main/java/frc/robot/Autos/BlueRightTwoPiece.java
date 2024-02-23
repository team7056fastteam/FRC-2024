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

public class BlueRightTwoPiece  extends FastAutoBase{
    //{x,y,heading,error}
    double[][] point0 = {{-5.58,40.21,314.69,3}};
    double[][] point1 = {{-2.85,80.67,0,3.5}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));

        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new RunPathCommand(path1));
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
        
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }
    
}
