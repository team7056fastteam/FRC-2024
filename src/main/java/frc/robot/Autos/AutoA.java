package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Autos.Common.FastCommandScheduler;
import frc.robot.Commands.RunPathCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoA {
    //{x,y,heading,error}
    static double[][] path0 = {{-0.7,2.0,0,0.2}};

    static double[][] path1 = {{ -.7,9.2,0,0.2}};

    static double[][] path2 = {{5.98,5.41,0,0.15}};

    static double[][][] paths = {path0, path1, path2};

    public static Boolean onetime;

    public static void runAutonomousA(Pose2d pose, SwerveSubsystem _drive){
        if(!onetime){
            FastCommandScheduler.ScheduleCommand(new RunPathCommand(_drive, paths, pose, 0));
            onetime = true;
        }
    }
}
