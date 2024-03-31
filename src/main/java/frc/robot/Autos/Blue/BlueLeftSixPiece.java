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

public class BlueLeftSixPiece extends FastAutoBase{
    double[][] point0 = {KurtMath.addXYToPoint(FieldLayout.blueFirstNote, 16, -35)};
    double[][] noteOne = {FieldLayout.blueFirstNote};
    double[][] noteTwo = {KurtMath.addXYToPoint(FieldLayout.blueSecondNote, -20, -24),FieldLayout.blueSecondNote};
    double[][] noteThree = {KurtMath.addXYToPoint(FieldLayout.blueThirdNote, -26, -24),KurtMath.addXYToPoint(FieldLayout.blueThirdNote, -10, -5)};
    double[][] noteFourTo = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueStageLeft, 0, -38),3.5,14)
        , KurtMath.convertToVelocity(FieldLayout.blueStageLeft,3.5,14)
        , KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueThirdNoteLong, 0, -40),3.5,14), FieldLayout.blueThirdNoteLong};
    double[][] LongNoteBack = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueThirdNoteLong, 0, -120),3.5,14), FieldLayout.blueStageLeft};
    double[][] noteFiveTo = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueFourthNoteLong, 0, -25),3.5,14), FieldLayout.blueFourthNoteLong};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(noteOne, WayPointBehavior.Standard);
    Path path2 = new Path(noteTwo, WayPointBehavior.Standard);
    Path path3 = new Path(noteThree, WayPointBehavior.Standard);
    Path path4 = new Path(noteFourTo, WayPointBehavior.Velocity);
    Path path5 = new Path(LongNoteBack, WayPointBehavior.Velocity);
    Path path6 = new Path(noteFiveTo, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new PivotCommand(pivotState.kHoming, 47));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.25));
        runCommand(new PivotCommand(pivotState.kPivoting, 35));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueFirstNote[0],FieldLayout.blueFirstNote[1], 10),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kForward),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueSecondNote[0],FieldLayout.blueSecondNote[1], 10),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueThirdNote[0]-10,FieldLayout.blueThirdNote[1] - 10, 10),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new PivotCommand(pivotState.kPivoting, 30));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueThirdNoteLong[0],FieldLayout.blueThirdNoteLong[1], 15),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new WaitForIntake(2),
                new KurtinatorCommand(KurtinatorState.kIdle),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueStageLeft[0],FieldLayout.blueStageLeft[1], 6),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path6),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueFourthNoteLong[0],FieldLayout.blueFourthNoteLong[1], 15),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward),
                new WaitForIntake(2),
                new KurtinatorCommand(KurtinatorState.kIdle),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.blueStageLeft[0],FieldLayout.blueStageLeft[1], 6),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(1));
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return FieldLayout.blueStartingLeft;
    }
    
}
