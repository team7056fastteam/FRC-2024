package frc.robot.Autos.Red;

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

public class RedLeftFourPieceLong extends FastAutoBase{
    double[][] noteOnePreload = {KurtMath.modifyAngle(KurtMath.addXYToPoint(FieldLayout.redThirdNote,-20,-40),53)};
    double[][] noteTwoLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFifthNoteLong,0,-50),3.5,16),FieldLayout.redFifthNoteLong};
    double[][] noteThreeLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFifthNoteLong,20,-120),3,20),FieldLayout.redFourthNoteLong};
    double[][] NoteFourLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redThirdNoteLong, 0, -40),3.5,16), FieldLayout.redThirdNoteLong};
    double[][] OneLongBackRight = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFifthNoteLong,0,-50),3.5,16),
        KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redShootingLeft,-20,50),3.5,14),FieldLayout.redShootingLeft};
    double[][] TwoLongBackRight = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redThirdNoteLong, 0, -120),3.5,16), FieldLayout.redStageRight};

    Path path0 = new Path(noteOnePreload, WayPointBehavior.Standard);
    Path path1 = new Path(noteTwoLong, WayPointBehavior.Velocity);
    Path path2 = new Path(noteThreeLong, WayPointBehavior.Velocity);
    Path path3 = new Path(OneLongBackRight, WayPointBehavior.Velocity);
    Path path4 = new Path(TwoLongBackRight, WayPointBehavior.Velocity);
    Path path5 = new Path(NoteFourLong, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new PivotCommand(pivotState.kHoming, 40));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new WaitCommand(0.25));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFifthNoteLong[0],FieldLayout.redFifthNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new PivotCommand(pivotState.kPivoting, 30));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redShootingLeft[0],FieldLayout.redShootingLeft[1], 4),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFourthNoteLong[0],FieldLayout.redFourthNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redStageRight[0],FieldLayout.redStageRight[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redThirdNoteLong[0],FieldLayout.redThirdNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redStageRight[0],FieldLayout.redStageRight[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return FieldLayout.redStartingLeft;
    }
    
}
