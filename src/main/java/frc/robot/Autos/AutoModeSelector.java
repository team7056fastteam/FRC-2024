package frc.robot.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.Blue.*;
import frc.robot.Autos.Red.*;
import frc.robot.Common.FastAutoBase;

public class AutoModeSelector {
    private static FastAutoBase BlueLeftSixPiece = new BlueLeftSixPiece();
    private static FastAutoBase BlueLeftFourPieceLong = new BlueLeftFourPieceLong();
    private static FastAutoBase BlueRightFourPieceLong = new BlueRightFourPieceLong();
    private static FastAutoBase BlueLeftFourPiece = new BlueLeftFourPiece();
    private static FastAutoBase BlueRightThreePiece = new BlueRightThreePiece();
    private static FastAutoBase BlueRightThreepieceLong = new BlueRightThreepieceLong();
    private static FastAutoBase BlueMidLeftThreePiece = new BlueMidLeftThreePiece();
    private static FastAutoBase BlueMidRightThreePiece = new BlueMidRightThreePiece();
    private static FastAutoBase BlueRightThreePieceSuperLong = new BlueRightThreePieceSuperLong();
    private static FastAutoBase BlueMidLong = new BlueMidLong();
    private static FastAutoBase BlueLeftThreePieceLong = new BlueLeftThreePieceLong();
    private static FastAutoBase BlueLeftThreePiece = new BlueLeftThreePiece();
    private static FastAutoBase BlueLeftOnePiece = new BlueLeftOnePiece();
    private static FastAutoBase BlueRightOnePiece = new BlueRightOnePiece();
    private static FastAutoBase BlueGetScrewed = new BlueGetScrewed();

    private static FastAutoBase RedRightSixPiece = new RedRightSixPiece();
    private static FastAutoBase RedLeftFourPieceLong = new RedLeftFourPieceLong();
    private static FastAutoBase RedRightFourPieceLong = new RedRightFourPieceLong();
    private static FastAutoBase RedLeftThreePiece = new RedLeftThreePiece();
    private static FastAutoBase RedRightThreePiece = new RedRightThreePiece();
    private static FastAutoBase RedLeftThreePieceLongShiny = new RedLeftThreePieceLongShiny();
    private static FastAutoBase RedRightFourPiece = new RedRightFourPiece();
    private static FastAutoBase RedLeftThreePieceSuperLong = new RedLeftThreePieceSuperLong();
    private static FastAutoBase RedMidLeftThreePiece = new RedMidLeftThreePiece();
    private static FastAutoBase RedMidLong = new RedMidLong();
    private static FastAutoBase RedMidRightThreePiece = new RedMidRightThreePiece();
    private static FastAutoBase RedRightThreePieceLong = new RedRightThreePieceLong();
    private static FastAutoBase RedLeftOnePiece = new RedLeftOnePiece();
    private static FastAutoBase RedRightOnePiece = new RedRightOnePiece();
    private static FastAutoBase RedGetScrewed = new RedGetScrewed();

    private static FastAutoBase DoNothing = new DoNothing();

    //private static FastAutoBase Testing = new Testing();
    //private static FastAutoBase AutoB = new AutoB();
    private static SendableChooser<FastAutoBase> autoChooser = new SendableChooser<>();

    public AutoModeSelector(){
        //Auto Chooser
        autoChooser.setDefaultOption("Blue Side Left Four piece", BlueLeftFourPiece);

        autoChooser.addOption("Blue Side Left Four piece", BlueLeftFourPiece);
        autoChooser.addOption("Blue Funny", BlueGetScrewed);
        autoChooser.addOption("Blue Side Left Six piece", BlueLeftSixPiece);
        autoChooser.addOption("Blue Side Left Four piece long", BlueLeftFourPieceLong);
        autoChooser.addOption("Blue Side Right Four piece long", BlueRightFourPieceLong);
        autoChooser.addOption("Blue Side Left Three piece", BlueLeftThreePiece);
        autoChooser.addOption("Blue Side Left Three piece Long", BlueLeftThreePieceLong);
        autoChooser.addOption("Blue Side Right Three piece Long", BlueRightThreepieceLong);
        autoChooser.addOption("Blue Side Right Three piece Super Long", BlueRightThreePieceSuperLong);
        autoChooser.addOption("Blue Side Right Three piece", BlueRightThreePiece);
        autoChooser.addOption("Blue Side Mid Left Three Piece", BlueMidLeftThreePiece);
        autoChooser.addOption("Blue Side Middle Long", BlueMidLong);
        autoChooser.addOption("Blue Side Mid Right Three Piece", BlueMidRightThreePiece);
        autoChooser.addOption("Blue Side Left One Piece", BlueLeftOnePiece);
        autoChooser.addOption("Blue Side Right One Piece", BlueRightOnePiece);

        autoChooser.addOption("Red Side Right Four Piece", RedRightFourPiece);
        autoChooser.addOption("Red Funny", RedGetScrewed);
        autoChooser.addOption("Red Side Right Three Piece", RedRightThreePiece);
        autoChooser.addOption("Red Side Right Three Piece Long", RedRightThreePieceLong);
        autoChooser.addOption("Red Side Middle Long", RedMidLong);
        autoChooser.addOption("Red Side Left Three Piece Long", RedLeftThreePieceLongShiny);
        autoChooser.addOption("Red Side Left Three Piece Super Long", RedLeftThreePieceSuperLong);
        autoChooser.addOption("Red Side Left Three piece", RedLeftThreePiece);
        autoChooser.addOption("Red Side Mid Left Three Piece", RedMidLeftThreePiece);
        autoChooser.addOption("Red Side Mid Right Three Piece", RedMidRightThreePiece);
        autoChooser.addOption("Red Side Left One Piece", RedLeftOnePiece);
        autoChooser.addOption("Red Side Right One Piece", RedRightOnePiece);
        autoChooser.addOption("Red Side Right Six piece", RedRightSixPiece);
        autoChooser.addOption("Red Side Right Four piece long", RedRightFourPieceLong);
        autoChooser.addOption("Red Side Left Four piece long", RedLeftFourPieceLong);

        autoChooser.addOption("Do Nothing :)", DoNothing);

        //autoChooser.addOption("Testing", Testing);
        // autoChooser.addOption("Auto B", AutoB);
        SmartDashboard.putData("Auto choices", autoChooser);
    }

    public FastAutoBase getAutoMode(){
        return autoChooser.getSelected();
    }
}
