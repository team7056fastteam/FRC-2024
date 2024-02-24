package frc.robot.Autos.Common;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.*;

public class AutoModeSelector {
    private static FastAutoBase BlueLeftFourPiece = new BlueLeftFourPiece();
    private static FastAutoBase BlueRightTwoPiece = new BlueRightTwoPiece();
    private static FastAutoBase BlueRightThreepieceLong = new BlueRightThreepieceLong();
    private static FastAutoBase RedRightFourPiece = new RedRightFourPiece();
    private static FastAutoBase RedLeftThreePieceLong = new RedLeftThreePieceLong();
    //private static FastAutoBase AutoB = new AutoB();
    private static SendableChooser<FastAutoBase> autoChooser = new SendableChooser<>();

    public AutoModeSelector(){
        //Auto Chooser
        autoChooser.setDefaultOption("Blue Side Left Four piece", BlueLeftFourPiece);
        autoChooser.addOption("Blue Side Left Three piece", BlueLeftFourPiece);
        autoChooser.addOption("Blue Side Right Three piece Long", BlueRightThreepieceLong);
        autoChooser.addOption("Blue Side Right Two piece", BlueRightTwoPiece);

        autoChooser.addOption("Red Side Right Four Piece", RedRightFourPiece);
        autoChooser.addOption("Red Side Left Three piece Long", RedLeftThreePieceLong);
        // autoChooser.addOption("Auto B", AutoB);
        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public FastAutoBase getAutoMode(){
        return autoChooser.getSelected();
    }
}
