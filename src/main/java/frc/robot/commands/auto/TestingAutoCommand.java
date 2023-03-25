package frc.robot.commands.auto;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TestingAutoCommand extends CommandBase {
    public static Command getAutoMode() {
        String pathFile = "Testing";
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathFile, new PathConstraints(1.6, 0.6));
        
        PathPlannerTrajectory path1 = path.get(0);
        HashMap<String, Command> pathEventMap1 = new HashMap<>();

        FollowPathWithEvents path1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path1), 
            path1.getMarkers(),
            pathEventMap1);

        PathPlannerTrajectory path2 = path.get(1);
        HashMap<String, Command> pathEventMap2 = new HashMap<>();

        FollowPathWithEvents path2WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path2), 
            path2.getMarkers(),
            pathEventMap2);

        Command mobilityCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path1)
        .andThen(path1WithEvents)
        .andThen(path2WithEvents); 

        return mobilityCommand;
    }
}
