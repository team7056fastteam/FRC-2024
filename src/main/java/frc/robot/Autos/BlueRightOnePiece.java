package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AutoCommands.*;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class BlueRightOnePiece extends FastAutoBase{
    double[][] point0 = {{-5.58,40.21,314.69,3}};

    Path path0 = new Path(point0, WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new SetGoalTranslation(new Translation2d(-60,0)));
        runCommand(new ShooterCommand(shooterState.kHigh));
        runCommand(new RunPathCommand(path0));
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new WaitCommand(3));
        runCommand(new StopCommand());
    }
    
    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0,0, Rotation2d.fromRadians(0));
    }
}
