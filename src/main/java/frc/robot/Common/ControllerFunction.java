package frc.robot.Common;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerFunction {
    XboxController driver, operator;
    public ControllerFunction(XboxController driver, XboxController operator){
        this.driver = driver;
        this.operator = operator;
    }
    /**Driver Left Bumper*/
    public boolean speedAdjustment(){
        return driver.getLeftBumper();
    }
    /**Driver Y Button*/
    public boolean Target(){
        return driver.getYButton();
    }
    /**Driver A Button*/
    public boolean Reset(){
        return driver.getAButton();
    }
    /**Driver D-Pad Up*/
    public boolean ResetYaw(){
        return driver.getPOV() == 0;
    }
    /**Driver Right Bumper*/
    public boolean AngleLock(){
        return driver.getRightBumper();
    }
    /**Driver Left Trigger*/
    public boolean lockWheels(){
        return driver.getRawAxis(2) > 0.1;
    }
    /**Driver Right Trigger*/
    public boolean robotOriented(){
        return driver.getRawAxis(3) > 0.1;
    }
    public double driverX(){
        return driver.getRawAxis(1) * -1;
    }
    public double driverY(){
        return driver.getRawAxis(0) * -1;
    }
    public double driverZ(){
        return driver.getRawAxis(4) * -1;
    }
    /**Operator A Button*/
    public boolean IngestIn(){
        return operator.getAButton();
    }
    /**Operator Y Button*/
    public boolean IngestOut(){
        return operator.getYButton();
    }
    /**Operator B Button*/
    public boolean LowShot(){
        return operator.getBButton();
    }
    /**Operator X Button*/
    public boolean HighShot(){
        return operator.getXButton();
    }
    /**Operator Right Bumper*/
    public boolean Feed(){
        return operator.getRightBumper();
    }
    /**Operator Left Bumper*/
    public boolean Climb(){
        return operator.getLeftBumper();
    }
    /**Operator Left Trigger*/
    public boolean UnClimb(){
        return operator.getRawAxis(2) > 0.1;
    }
    /**Operator POV Up*/
    public boolean Flipp(){
        return operator.getPOV() == 0;
    }
    /**Operator POV Down*/
    public boolean UnFlipp(){
        return operator.getPOV() == 180;
    }

    public void Button(boolean active, FastTeleOpAction action){
        if(active){
            action.run();
        }
    }
}
