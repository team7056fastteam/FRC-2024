package frc.robot.Autos;

import java.util.List;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Autos.Common.Line;
import frc.robot.Autos.Common.NewPath;
import frc.robot.Autos.Common.Point;
//import frc.robot.Autos.Common.Line.LineType;
import frc.robot.Autos.Common.NewPath.WayPointBehavior;
import frc.robot.Commands.NewPathCommand;

public class Testing extends FastAutoBase{
    Line line0 = new Line(new Point(0,0,0), new Point(24,48,0));
    Line line1 = new Line(new Point(24,48,0), new Point(36,36,0));
    NewPath path0 = new NewPath(List.of(line0, line1), WayPointBehavior.Standard);

    @Override
    public void routine() throws Exception {
        runCommand(new NewPathCommand(path0));
    }
}
