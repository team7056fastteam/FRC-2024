package frc.robot.Autos.Red;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AutoCommands.*;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.FastParallel;
import frc.robot.Common.FastSeries;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class RedRightThreePieceLong extends FastAutoBase{
    //{x,y,heading,error} 
    double[][] point0 = {{0,38.84,315.63,3}};
    double[][] point1 = {{4.56,85,0,3}, {0,38.84,315.63,3}};
    double[][] point2 = {{14.51, 249.64, 0, 14, 3}, {14.51, 297.64, 0, 3}};
    double[][] point3 = {{14.51, 105.21,0,14, 3},{0,38.84,315.63,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path2 = new Path(point2, WayPointBehavior.Velocity);
    Path path3 = new Path(point3, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new SetGoalTranslation(new Translation2d(-60,0)));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(4.56, 85, 10),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(0, 38.8, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(14.51, 249.64, 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(0, 38.8, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0,0, Rotation2d.fromRadians(0));
    }
}
