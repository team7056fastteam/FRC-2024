package frc.robot.Autos.Common;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.AutoA;

public class AutoModeSelector {
    private static FastAutoBase AutoA = new AutoA();
    //private static FastAutoBase AutoB = new AutoB();
    private static SendableChooser<FastAutoBase> autoChooser = new SendableChooser<>();

    public AutoModeSelector(){
        //Auto Chooser
        autoChooser.setDefaultOption("Auto A", AutoA);
        autoChooser.addOption("Auto A", AutoA);
        // autoChooser.addOption("Auto B", AutoB);
        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public FastAutoBase getAutoMode(){
        return autoChooser.getSelected();
    }
}
