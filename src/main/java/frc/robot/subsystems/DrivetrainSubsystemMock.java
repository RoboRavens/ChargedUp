package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainSubsystemMock extends DrivetrainSubsystemBase {

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {}

    @Override
    public Rotation2d getOdometryRotation() {
        return new Rotation2d();
    }


    @Override
    public void holdPosition() {}

    @Override
    public void zeroGyroscope() {}

    @Override
    public void cutPower() {}

    @Override
    public void stopCutPower() {}

    @Override
    public boolean powerIsCut() {
        return false;
    }

    @Override
    public TrajectoryConfig GetTrajectoryConfig() {
        return new TrajectoryConfig(0,0);
    }

    @Override
    public Command CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory trajectory) {
        return new InstantCommand();
    }

    @Override
    public Command CreateFollowTrajectoryCommand(Trajectory trajectory) {
        return new InstantCommand();
    }

    @Override
    public Command CreateFollowTrajectoryCommandSwerveOptimized(Trajectory trajectory) {
        return new InstantCommand();
    }

    @Override
    public Command getMarkPositionCommand() {
        return new InstantCommand();
    }

    @Override
    public Command getReturnToMarkedPositionCommand() {
        return new InstantCommand();
    }
    
    @Override
    public Rotation2d getGyroscopeRotation2dTest() {
       return new Rotation2d();
        
    }
}
