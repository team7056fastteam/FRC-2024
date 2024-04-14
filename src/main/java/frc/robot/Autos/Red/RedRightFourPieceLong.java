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

public class RedRightFourPieceLong extends FastAutoBase{
    double[][] noteOne = {KurtMath.addXYToPoint(FieldLayout.redFirstNote, -16, -35)};
    double[][] noteTwo = {FieldLayout.redFirstNote};
    double[][] noteThree = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFirstNoteLong, 0, -50),3.5,14),FieldLayout.redFirstNoteLong};
    double[][] shootLong = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redRightWingShot, 15, 20), 3.5, 14), FieldLayout.redRightWingShot};
    double[][] noteFour = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redSecondNoteLong, 20, -60),3.5,14),FieldLayout.redSecondNoteLong};

    Path path0 = new Path(noteOne, WayPointBehavior.Standard);
    Path path1 = new Path(noteTwo, WayPointBehavior.Standard);
    Path path2 = new Path(noteThree, WayPointBehavior.Velocity);
    Path path3 = new Path(shootLong, WayPointBehavior.Velocity);
    Path path4 = new Path(noteFour, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new PivotCommand(pivotState.kHoming, 42));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(0.25));
        runCommand(new PivotCommand(pivotState.kPivoting, 35));
        runCommand(new WaitCommand(0.25));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path1),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFirstNote[0],FieldLayout.redFirstNote[1], 25),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kForward),
                new WaitForIntake(1)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path2),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redFirstNoteLong[0],FieldLayout.redFirstNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new PivotCommand(pivotState.kPivoting, 30));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redRightWingShot[0],FieldLayout.redRightWingShot[1], 3),
                new KurtinatorCommand(KurtinatorState.kFeed),
                new IngestCommand(IngestState.kIdle)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path4),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redSecondNoteLong[0],FieldLayout.redSecondNoteLong[1], 20),
                new KurtinatorCommand(KurtinatorState.kRunTilTrip),
                new IngestCommand(IngestState.kForward)
        )))));
        runCommand(new WaitCommand(0.1));
        runCommand(new FastParallel(List.of(
            new RunPathCommand(path3),
            new FastSeries(List.of(
                new PassXYCommand(FieldLayout.redRightWingShot[0],FieldLayout.redRightWingShot[1], 3),
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
