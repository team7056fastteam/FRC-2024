package frc.robot.Autos.Red;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldLayout;
import frc.robot.AutoCommands.RunPathCommand;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.KurtMath;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;

public class RedGetScrewed extends FastAutoBase{
    double[][] NoteFarOne = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.redFirstNoteLong, 0, -20),3,20),FieldLayout.redFirstNoteLong};
    double[][] notefarer = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(KurtMath.modifyAngle(FieldLayout.redFifthNoteLong,45),20,0),3,20),KurtMath.modifyAngle(FieldLayout.redFifthNoteLong,45)};

    Path path0 = new Path(NoteFarOne, WayPointBehavior.Velocity);
    Path path1 = new Path(notefarer, WayPointBehavior.Velocity);

    @Override
    public void routine() throws Exception {
        runCommand(new RunPathCommand(path0));
        runCommand(new RunPathCommand(path1));
    }

    @Override
    public Pose2d getStartingPose() {
        return FieldLayout.blueStartingLeft;
    }
    
}
