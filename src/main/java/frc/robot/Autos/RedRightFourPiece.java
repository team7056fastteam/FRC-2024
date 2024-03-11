package frc.robot.Autos;

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

public class RedRightFourPiece extends FastAutoBase{
    //{x,y,heading,error} 
    double[][] point0 = {{8.53,26.80,297.63,3}};
    double[][] point1 = {{4.56,85.71,0,3}, {-50.59,58.58,0,3}};
    double[][] point2 = {{-50.59, 87.51,0,3},{-89.68, 60.07,30,3}};
    double[][] point3 = {{-106.47,87.29, 30, 3}, {-89.68, 60.07,26,3}};
    double[][] point4 = {{ -27.71, 100.54, 0, 10, 2.5},{-110.64, 192.43, 0, 10, 2.5}, {-110.64, 286.15, 0, 10, 2.5},{-110.64, 296.15, 0, 3}};
    double[][] point5 = {{-110.64, 192.43, 0, 10, 2.5}, {-44.63, 137.87, 0, 10, 2.5}, { -52.95, 61.50, 0 ,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path2 = new Path(point2, WayPointBehavior.Standard);
    Path path3 = new Path(point3, WayPointBehavior.Standard);
    Path path4 = new Path(point4, WayPointBehavior.Velocity);
    Path path5 = new Path(point5, WayPointBehavior.Velocity);

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
                new PassXYCommand(4.56, 85.71, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-50.59, 58.58, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(-50.59, 87.51, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-89.68, 60.07, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(-106.47,87.29, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-89.68, 60.07, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(-110.64, 286.15, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(-52.95, 61.50, 7),
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
