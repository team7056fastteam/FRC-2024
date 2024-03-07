package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Autos.Common.FastCommand;
import frc.robot.Autos.Common.Path;
import frc.robot.Autos.Common.pathRunnerHeadAtPoint;
import frc.robot.Constants.DriveConstants;

public class RunPathHeadAtPointCommand extends FastCommand{
    pathRunnerHeadAtPoint _path = new pathRunnerHeadAtPoint();

    ChassisSpeeds targetChassisSpeeds;
    Pose2d currentPose;

    Path path;

    double x=0, y=0;

    public RunPathHeadAtPointCommand(Path path, double x, double y){
        this.path = path;
        this.x = x;
        this.y = y;
    }

    @Override
    public void run() {
        if(Robot.getPose() != null){
            Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(
            _path.runpath(Robot.getPose(), path, x, y)));
        }
    }

    @Override
    public Boolean isFinished() {
        return pathRunnerHeadAtPoint.kStopPath;
    }

    @Override
    public void init() {
        pathRunnerHeadAtPoint.kStopPath = false;
        pathRunnerHeadAtPoint.selectedPoint = 0;
        pathRunnerHeadAtPoint.highestXPower = 0;
        pathRunnerHeadAtPoint.highestYPower = 0;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        
    }

    @Override
    public void end() {
        pathRunnerHeadAtPoint.kStopPath = true;
        targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Robot.getPose().getRotation());
        Robot._drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));
    }
    
}
