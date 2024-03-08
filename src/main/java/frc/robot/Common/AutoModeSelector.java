package frc.robot.Common;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.*;

public class AutoModeSelector {
    private static FastAutoBase BlueLeftFourPiece = new BlueLeftFourPiece();
    private static FastAutoBase BlueRightThreePiece = new BlueRightThreePiece();
    private static FastAutoBase BlueRightThreepieceLong = new BlueRightThreepieceLong();
    private static FastAutoBase BlueMidLeftThreePiece = new BlueMidLeftThreePiece();
    private static FastAutoBase BlueMidRightThreePiece = new BlueMidRightThreePiece();
    private static FastAutoBase BlueLeftThreePieceLong = new BlueLeftThreePieceLong();
    private static FastAutoBase BlueLeftThreePiece = new BlueLeftThreePiece();
    private static FastAutoBase BlueLeftOnePiece = new BlueLeftOnePiece();
    private static FastAutoBase BlueRightOnePiece = new BlueRightOnePiece();

    private static FastAutoBase RedLeftThreePiece = new RedLeftThreePiece();
    private static FastAutoBase RedRightThreePiece = new RedRightThreePiece();
    private static FastAutoBase RedRightThreePieceLong = new RedRightThreePieceLong();
    private static FastAutoBase RedRightFourPiece = new RedRightFourPiece();
    private static FastAutoBase RedLeftThreePieceLong = new RedLeftThreePieceLong();
    private static FastAutoBase RedMidLeftThreePiece = new RedMidLeftThreePiece();
    private static FastAutoBase RedMidRightThreePiece = new RedMidRightThreePiece();
    private static FastAutoBase RedLeftOnePiece = new RedLeftOnePiece();
    private static FastAutoBase RedRightOnePiece = new RedRightOnePiece();

    private static FastAutoBase DoNothing = new DoNothing();

    //private static FastAutoBase Testing = new Testing();
    //private static FastAutoBase AutoB = new AutoB();
    private static SendableChooser<FastAutoBase> autoChooser = new SendableChooser<>();

    public AutoModeSelector(){
        //Auto Chooser
        autoChooser.setDefaultOption("Blue Side Left Four piece", BlueLeftFourPiece);
        autoChooser.addOption("Blue Side Left Four piece", BlueLeftFourPiece);
        autoChooser.addOption("Blue Side Left Three piece", BlueLeftThreePiece);
        autoChooser.addOption("Blue Side Left Three piece Long", BlueLeftThreePieceLong);
        autoChooser.addOption("Blue Side Right Three piece Long", BlueRightThreepieceLong);
        autoChooser.addOption("Blue Side Right Three piece", BlueRightThreePiece);
        autoChooser.addOption("Blue Side Mid Left Three Piece", BlueMidLeftThreePiece);
        autoChooser.addOption("Blue Side Mid Right Three Piece", BlueMidRightThreePiece);
        autoChooser.addOption("Blue Side Left One Piece", BlueLeftOnePiece);
        autoChooser.addOption("Blue Side Right One Piece", BlueRightOnePiece);

        autoChooser.addOption("Red Side Right Four Piece", RedRightFourPiece);
        autoChooser.addOption("Red Side Right Three Piece", RedRightThreePiece);
        autoChooser.addOption("Red Side Right Three Piece Long", RedRightThreePieceLong);
        autoChooser.addOption("Red Side Left Three piece Long", RedLeftThreePieceLong);
        autoChooser.addOption("Red Side Left Three piece", RedLeftThreePiece);
        autoChooser.addOption("Red Side Mid Left Three Piece", RedMidLeftThreePiece);
        autoChooser.addOption("Red Side Mid Right Three Piece", RedMidRightThreePiece);
        autoChooser.addOption("Red Side Left One Piece", RedLeftOnePiece);
        autoChooser.addOption("Red Side Right One Piece", RedRightOnePiece);

        autoChooser.addOption("Do Nothing :)", DoNothing);

        //autoChooser.addOption("Testing", Testing);
        // autoChooser.addOption("Auto B", AutoB);
        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public FastAutoBase getAutoMode(){
        return autoChooser.getSelected();
    }
}
