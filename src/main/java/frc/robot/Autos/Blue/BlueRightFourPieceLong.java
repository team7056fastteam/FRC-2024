package frc.robot.Autos.Blue;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AutoCommands.*;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.FastParallel;
import frc.robot.Common.FastSeries;
import frc.robot.Common.KurtMath;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;
import frc.robot.Constants.FieldLayout;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.Pivotinator.pivotState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightFourPieceLong extends FastAutoBase{
    double[][] noteOnePreload = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueShootingRight,-20,-20),3.5,16),FieldLayout.blueShootingRight};
    double[][] noteTwoLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueFifthNoteLong,0,-50),3.5,16),FieldLayout.blueFifthNoteLong};
    double[][] noteThreeLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueFifthNoteLong,-20,-120),3,20),FieldLayout.blueFourthNoteLong};
    double[][] NoteFourLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueThirdNoteLong, 0, -40),3.5,16), FieldLayout.blueThirdNoteLong};
    double[][] OneLongBackRight = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueFifthNoteLong,0,-50),3.5,16),
        KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueShootingRight,20,50),3.5,14),FieldLayout.blueShootingRight};
    double[][] TwoLongBackRight = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueThirdNoteLong, 0, -120),3.5,16), FieldLayout.blueStageLeft};

    Path path0 = new Path(noteOnePreload, WayPointBehavior.Velocity);
    Path path1 = new Path(noteTwoLong, WayPointBehavior.Velocity);
    Path path2 = new Path(noteThreeLong, WayPointBehavior.Velocity);
    Path path3 = new Path(OneLongBackRight, WayPointBehavior.Velocity);
    Path path4 = new Path(TwoLongBackRight, WayPointBehavior.Velocity);
    Path path5 = new Path(NoteFourLong, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new PivotCommand(pivotState.kHoming, 30));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new WaitCommand(0.25));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueFifthNoteLong[0],FieldLayout.blueFifthNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueShootingRight[0],FieldLayout.blueShootingRight[1], 4),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueFourthNoteLong[0],FieldLayout.blueFourthNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueStageLeft[0],FieldLayout.blueStageLeft[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueThirdNoteLong[0],FieldLayout.blueThirdNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueStageLeft[0],FieldLayout.blueStageLeft[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
    }

    @Override
    public Pose2d getStartingPose() {
        return FieldLayout.blueStartingRight;
    }
    
}
