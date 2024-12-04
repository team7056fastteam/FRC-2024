package frc.robot.Autos.Blue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldLayout;
import frc.robot.AutoCommands.RunPathCommand;
import frc.robot.Common.FastAutoBase;
import frc.robot.Common.KurtMath;
import frc.robot.Common.Path;
import frc.robot.Common.Path.WayPointBehavior;

public class BlueGetScrewed extends FastAutoBase{
    double[][] NoteFarOne = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(FieldLayout.blueFirstNoteLong, 0, -20),3,20),FieldLayout.blueFirstNoteLong};
    double[][] notefarer = {KurtMath.convertToVelocity(KurtMath.addXYToPoint(KurtMath.modifyAngle(FieldLayout.blueFifthNoteLong,45),-20,0),3,20),KurtMath.modifyAngle(FieldLayout.blueFifthNoteLong,45)};

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
