package frc.robot.Autos.Blue;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.AutoCommands.*;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.FastParallel;
import frc.robot.Common.FastSeries;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightThreepieceLongShiny extends FastAutoBase{
    double[][] point0 = {{-5.58,40.21,314.69,3}};
    double[][] point1 = {{-2.85,80.67,0,3.5}, {-5.58,40.21,314.69,5}};
    double[][] point2 = {{-119.25, 164.75,0,16,3},{-113, 188,0,3}};
    double[][] firstNote = {{133.95,297.89,0,3}};
    double[][] secondNote = {{67.95,297.89,0,3}};
    double[][] point3 = {{109.94,170.56,0,16,3}, {20.53,46.80,48,16,3},{-4.97, 41.97, 314.69,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(point1, WayPointBehavior.Standard);
    Path path2 = new Path(point2, WayPointBehavior.Velocity);
    Path path3 = new Path(point3, WayPointBehavior.Velocity);
    Path first = new Path(firstNote, WayPointBehavior.Standard);
    Path second = new Path(secondNote, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new SetGoalTranslation(new Translation2d(-60,0)));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(122, 221, 8),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.25));
        if(Robot.getTx() > 0){
            runCommand(new RunPathCommand(first));
        }
        else{
            runCommand(new RunPathCommand(second));
        }
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(-5.5, 40.21, 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.25));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(-2, 80.71, 8),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new PassXYCommand(-5.5, 40.21, 3),
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
