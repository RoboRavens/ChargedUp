package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.util.AutoMode;

public class ScoreTwoLoadAndBalanceCommand {
    public static AutoMode getAutoMode() {
        // List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup("2056 2.5 + Balance", new PathConstraints(1.6, 0.6));
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("2056 2.5 + Balance", new PathConstraints(1.6, 0.6));

        // PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = examplePath.get(0);
        // HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();

        // PathPlannerTrajectory loadCone1ToScoreMidTrajectory = examplePath.get(1);
        // PathPlannerTrajectory scoreMidToLoadCone2Trajectory = examplePath.get(2);
        // PathPlannerTrajectory loadCone2ToBridgeTrajectory = examplePath.get(3);

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Load Cone 1", new TempAutoIntakeCommand());
        // eventMap.put("Score Cone Mid", new TempAutoIntakeCommand());
        // eventMap.put("Load Cone 2", new TempAutoIntakeCommand());

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(),
            eventMap);

        Command scoreTwoLoadAndBalanceCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(examplePathWithEvents);

        return new AutoMode("Score Two Pieces, Load, and Balance Auto", scoreTwoLoadAndBalanceCommand);
    }
}
