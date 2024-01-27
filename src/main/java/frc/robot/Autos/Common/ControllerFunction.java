package frc.robot.Autos.Common;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerFunction {
    XboxController driver, operator;
    public ControllerFunction(XboxController driver, XboxController operator){
        this.driver = driver;
        this.operator = operator;
    }
    public boolean speedAdjustment(){
        return driver.getLeftBumper();
    }
    public boolean Target(){
        return driver.getYButton();
    }
    public boolean Reset(){
        return driver.getAButton();
    }
    public boolean AngleLock(){
        return driver.getRightBumper();
    }
    //Left Trigger
    public boolean lockWheels(){
        return driver.getRawAxis(2) > 0.1;
    }
    //Right Trigger
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
}
