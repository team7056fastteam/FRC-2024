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

public class BlueRightThreepieceLong extends FastAutoBase{
    double[][] point0 = {{-5.58,40.21,314.69,3}};
    double[][] point1 = {{-2.85,80.67,0,3.5}, {-5.58,40.21,314.69,5}};
    double[][] point2 = {{60.83, 65.49,0,14,3},{122.61, 221.11,0,14,3},{126.73,296.49,0,14,3},{126.73,302.49,0,3}};
    double[][] point3 = {{126.73,296.49,0,14,3},{122.61, 221.11,0,14,3},{60.83, 65.49,0,14,3},{-5.58,40.21,314.69,3}};

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
                new PassXYCommand(-2.85, 80.5, 7),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-5.5, 40.21, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(122, 221, 8),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(-5.5, 40.21, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.5));
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0,0, Rotation2d.fromRadians(0));
    }
}
