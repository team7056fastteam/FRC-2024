package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AutoCommands.StopCommand;
import frc.robot.Common.FastAutoBase;

public class DoNothing extends FastAutoBase{
    @Override
    public void routine() throws Exception {
        runCommand(new StopCommand());
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(0,0, Rotation2d.fromRadians(0));
    }
}
