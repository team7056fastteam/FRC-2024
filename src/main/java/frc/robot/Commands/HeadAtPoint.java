package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;

public class HeadAtPoint extends FastCommand{
    double x=0,y=0;
    ChassisSpeeds targetChassisSpeeds;
    
    public HeadAtPoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    @Override
    public void init() {}

    @Override
    public void run() {
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }

    @Override
    public Boolean isFinished() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isFinished'");
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'end'");
    }
    
}
