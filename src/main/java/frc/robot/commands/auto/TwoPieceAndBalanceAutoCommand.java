package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.util.AutoMode;

public class TwoPieceAndBalanceAutoCommand {
    public static AutoMode getAutoMode() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Two Piece Auto and Balance", new PathConstraints(1.6, 0.6));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Retrieve Game Piece", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        eventMap.put("Score Game Piece", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(), 
            eventMap);

        Command testCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(new WaitCommand(1).deadlineWith(new ScorePieceCommand()))
        .andThen(examplePathWithEvents)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Test Auto", testCommand);
    }
}
