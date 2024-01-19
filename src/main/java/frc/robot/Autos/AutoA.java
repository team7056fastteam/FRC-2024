package frc.robot.Autos;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Commands.*;

public class AutoA extends FastAutoBase{
    //{x,y,heading,error}
    double[][] path0 = {{-0.7,2.0,0,1}};

    double[][] path1 = {{ -.7,9.2,0,0.2}};

    double[][] path2 = {{5.98,5.41,0,0.15}};

    double[][][] paths = {path0, path1, path2};

    @Override
    public void routine() throws Exception {
        runCommand(new RunPathCommand(paths, 0));
        runCommand(new WaitCommand(5));
        runCommand(new RunPathCommand(paths, 1));
    }
}
