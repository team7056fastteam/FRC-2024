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

public class RedLeftThreePieceSuperLong extends FastAutoBase{
    //{x,y,heading,error} 
    double[][] point0 = {{4.97, 41.97, 48,3}};
    double[][] point1 = {{-4.56,80.71,0,3}, {4.97, 41.97, 48,3}};
    double[][] firstNote = {{-100.25, 164.75,0,16,5},{-113, 188,0,16,4},{-133.95,297.89,0,3}};
    double[][] secondNote = {{-90.25, 164.75,0,16,5},{-90, 188,0,16,4},{-67.95,297.89,0,3}};
    double[][] point3 = {{-95.94,170.56,0,16,5}, {-20.53,50.80,48,16,3.5},{4.97, 41.97, 48,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path3 = new Path(point3, WayPointBehavior.Velocity);
    Path first = new Path(firstNote, WayPointBehavior.Velocity);
    Path second = new Path(secondNote, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new SetGoalTranslation(new Translation2d(60,0)));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(first),
            new FastSeries(List.of(
                new PassXYCommand(-113.62, 188.62, 16),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(4.97, 41.97, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(second),
            new FastSeries(List.of(
                new PassXYCommand(-90.62, 188.62, 16),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(4.97, 41.97, 4),
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
