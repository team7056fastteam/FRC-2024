package frc.robot.Autos;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Commands.StopCommand;

public class DoNothing extends FastAutoBase{
    @Override
    public void routine() throws Exception {
        runCommand(new StopCommand());
    }
}
