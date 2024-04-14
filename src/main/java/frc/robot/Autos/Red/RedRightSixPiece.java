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

public class RedRightSixPiece extends FastAutoBase{
    double[][] point0 = {KurtMath.addXYToPoint(FieldLayout.redFirstNote, -16, -35)};
    double[][] noteOne = {FieldLayout.redFirstNote};
    double[][] noteTwo = {KurtMath.addXYToPoint(FieldLayout.redSecondNote, 20, -24),FieldLayout.redSecondNote};
    double[][] noteThree = {KurtMath.addXYToPoint(FieldLayout.redThirdNote, 26, -24),KurtMath.addXYToPoint(FieldLayout.redThirdNote, 10, -5)};
    double[][] noteFourTo = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redStageRight, 0, -38),3.5,14)
        , KurtMath.convertToVelocity(FieldLayout.redStageRight,3.5,14)
        , KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redThirdNoteLong, 0, -40),3,14), FieldLayout.redThirdNoteLong};
    double[][] LongNoteBack = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redThirdNoteLong, 0, -120),3.5,14), FieldLayout.redStageRight};
    double[][] noteFiveTo = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFourthNoteLong, 0, -25),3.5,14), FieldLayout.redFourthNoteLong};

    Path path0 = new Path(point0, WayPointBehavior.Standard);
    Path path1 = new Path(noteOne, WayPointBehavior.Standard);
    Path path2 = new Path(noteTwo, WayPointBehavior.Standard);
    Path path3 = new Path(noteThree, WayPointBehavior.Standard);
    Path path4 = new Path(noteFourTo, WayPointBehavior.Velocity);
    Path path5 = new Path(LongNoteBack, WayPointBehavior.Velocity);
    Path path6 = new Path(noteFiveTo, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new PivotCommand(pivotState.kHoming, 42));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.25));
        runCommand(new PivotCommand(pivotState.kPivoting, 35));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFirstNote[0],FieldLayout.redFirstNote[1], 10),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kForward),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redSecondNote[0],FieldLayout.redSecondNote[1], 10),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redThirdNote[0] + 10,FieldLayout.redThirdNote[1] - 10, 10),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new PivotCommand(pivotState.kPivoting, 30));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redThirdNoteLong[0],FieldLayout.redThirdNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redStageRight[0],FieldLayout.redStageRight[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path6),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFourthNoteLong[0],FieldLayout.redFourthNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path5),
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
        return FieldLayout.redStartingRight;
    }
    
}
