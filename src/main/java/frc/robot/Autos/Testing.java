package frc.robot.Autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AutoCommands.NewPathCommand;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.Line;
import frc.robot.Common.NewPath;
import frc.robot.Common.Point;
import frc.robot.Common.NewPath.WayPointBehavior;

public class Testing extends FastAutoBase{
    Line line0 = new Line(new Point(0,0,0), new Point(100,0,0));
    //Line line1 = new Line(new Point(24,48,0), new Point(36,36,0));
    NewPath path0 = new NewPath(List.of(line0), WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new NewPathCommand(path0));
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0,0, Rotation2d.fromRadians(0));
    }
}
