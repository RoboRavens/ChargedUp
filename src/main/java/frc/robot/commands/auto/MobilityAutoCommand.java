package frc.robot.commands.auto;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class MobilityAutoCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance) {
        String pathFile = "Preload Cone + Mobility " + autoAlliance.toString();
        PathPlannerTrajectory path = PathPlanner.loadPath(pathFile, new PathConstraints(1.6, 0.6));
        
        HashMap<String, Command> pathEventMap = new HashMap<>();

        FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path), 
            path.getMarkers(),
            pathEventMap);

        Command pathCommand = pathWithEvents;


        Command mobilityCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path)
        .andThen(pathCommand).withTimeout(10); 

        return mobilityCommand;
    }
}
