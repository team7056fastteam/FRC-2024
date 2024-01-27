package frc.robot.Autos;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Commands.*;

public class AutoA extends FastAutoBase{
    //{x,y,heading,error}
    double[][] path0 = {{-11.39,28.10,61.81,2}};
    double[][] path1 = {{-1.56,73.71,0,1}};
    double[][] path2 = {{54.59,56.58,0,2}};
    double[][] path3 = {{50.34,73.62,0,1}};

    double[][][] paths = {path0, path1, path2, path3};

    @Override
    public void routine() throws Exception {
        runCommand(new RunPathCommand(paths, 0));
        runCommand(new WaitCommand(1));
        runCommand(new RunPathCommand(paths, 1));
        runCommand(new WaitCommand(1));
        runCommand(new RunPathCommand(paths, 2));
        runCommand(new WaitCommand(1));
        runCommand(new RunPathCommand(paths, 3));
        //runCommand(new TurnCommand());
    }
}
