package frc.robot.Autos;

import frc.robot.Autos.Common.FastAutoBase;
import frc.robot.Commands.*;
import frc.robot.subsystems.Specops.Ingest.IngestState;
import frc.robot.subsystems.Specops.Kurtinator.KurtinatorState;
import frc.robot.subsystems.Specops.ShootingSolution.shooterState;

public class AutoA extends FastAutoBase{
    //{x,y,heading,error}
    double[][] path0 = {{-11.39,28.10,61.81,2}};
    double[][] path1 = {{-1.56,73.71,0,1}};
    double[][] path2 = {{54.59,56.58,0,2}};
    double[][] path3 = {{50.34,73.62,0,1}};

    double[][][] paths = {path0, path1, path2, path3};

    @Override
    public void routine() throws Exception {
        //follow path
        runCommand(new RunPathCommand(paths, 0));
        //Shoot
        runCommand(new KurtinatorCommand(KurtinatorState.kFeed));
        runCommand(new ShooterCommand(shooterState.kHigh));
        //Wait
        runCommand(new WaitCommand(0.5));
        //UnShoot
        runCommand(new KurtinatorCommand(KurtinatorState.kIdle));
        runCommand(new ShooterCommand(shooterState.kIdle));
        //follow path
        runCommand(new RunPathCommand(paths, 1));
        //drive forward and intake
        runCommand(new DriveCommand(0, 0.5, 0));
        runCommand(new IngestCommand(IngestState.kForward));
        runCommand(new KurtinatorCommand(KurtinatorState.kRunTilTrip));
        runCommand(new WaitForIntake(1));
        runCommand(new DriveCommand(0, 0, 0));
        //Stop Ingest
        runCommand(new IngestCommand(IngestState.kIdle));
        //follow path
        runCommand(new RunPathCommand(paths, 2));
        runCommand(new WaitCommand(1));
        runCommand(new RunPathCommand(paths, 3));
    }
}
