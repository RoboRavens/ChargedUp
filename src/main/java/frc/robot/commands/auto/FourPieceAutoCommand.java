package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.util.AutoMode;

public class FourPieceAutoCommand {
    public static AutoMode getAutoMode() {
        // At this velocity and acceleration, the robot is able to score all four pieces in 29.81 seconds according to PathFinder
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Four Piece Auto", new PathConstraints(2, 1));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Retrieve Piece 2", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        eventMap.put("Score Piece 2", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        eventMap.put("Retrieve Piece 3", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        eventMap.put("Score Piece 3", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        eventMap.put("Retrieve Piece 4", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));        

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(), 
            eventMap);

        Command testCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(new WaitCommand(1).deadlineWith(new ScorePieceCommand()))
        .andThen(examplePathWithEvents)
        .andThen(new WaitCommand(1).deadlineWith(new ScorePieceCommand()));

        return new AutoMode("Test Auto", testCommand);
    }
}
