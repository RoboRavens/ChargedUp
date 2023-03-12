package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.util.AutoMode;

public class PreloadAndBalanceCommand {
    public static AutoMode getAutoMode() {
        PathPlannerTrajectory scoringToBridgeTrajectory = PathPlanner.loadPath("Preload and Balance", new PathConstraints(1, 0.4));

        HashMap<String, Command> eventMap = new HashMap<>();

        FollowPathWithEvents scoringToBridgePath = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoringToBridgeTrajectory), 
            scoringToBridgeTrajectory.getMarkers(), 
            eventMap);

        Command preloadAndBalanceCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoringToBridgeTrajectory)
        .andThen(new InstantCommand().until(() -> Robot.isRobotReadyToScore())) // Do nothing until the robot is ready to score
        .andThen(new ScorePieceCommand()) // Replace with TempAutoIntakeCommand() for testing on the old robot
        .andThen(scoringToBridgePath)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Preload and Balance", preloadAndBalanceCommand);
    }
}
