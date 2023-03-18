package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.claw.AutoClawCloseCommand;
import frc.robot.commands.claw.AutoClawOpenCommand;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.util.AutoMode;

public class ScoreTwoLoadAndBalanceCommand {
    public static AutoMode getAutoMode() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2.5 Cone + Balance", new PathConstraints(1.6, 0.6));

        // Path 1
        PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = pathGroup.get(0);
        
        HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();
        // Setpoint may need to be changed to be on the opposite side of the robot
        scorePreloadHighToLoadCone1EventMap.put("Extend To Ground 1", new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));

        FollowPathWithEvents scorePreloadHighToLoadCone1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scorePreloadHighToLoadCone1Trajectory), 
            scorePreloadHighToLoadCone1Trajectory.getMarkers(),
            scorePreloadHighToLoadCone1EventMap);

        Command scorePreloadHighToLoadCone1Command = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scorePreloadHighToLoadCone1Trajectory)
        .andThen(scorePreloadHighToLoadCone1WithEvents);


        // Path 2
        PathPlannerTrajectory loadCone1ToScoreMidTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreMidEventMap = new HashMap<>();
        loadCone1ToScoreMidEventMap.put("Extend To Score Mid", new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT));

        FollowPathWithEvents loadCone1ToScoreMidWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreMidTrajectory), 
            loadCone1ToScoreMidTrajectory.getMarkers(),
            loadCone1ToScoreMidEventMap);

        Command loadCone1ToScoreMidCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone1ToScoreMidTrajectory)
        .andThen(loadCone1ToScoreMidWithEvents);


        // Path 3
        PathPlannerTrajectory scoreMidToLoadCone2Trajectory = pathGroup.get(2);
        
        HashMap<String, Command> scoreMidToLoadCone2EventMap = new HashMap<>();
        scoreMidToLoadCone2EventMap.put("Extend To Ground 2", new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));

        FollowPathWithEvents scoreMidToLoadCone2WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoreMidToLoadCone2Trajectory), 
            scoreMidToLoadCone2Trajectory.getMarkers(),
            scoreMidToLoadCone2EventMap);

        Command scoreMidToLoadCone2Command = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoreMidToLoadCone2Trajectory)
        .andThen(scoreMidToLoadCone2WithEvents);


        // Path 4
        PathPlannerTrajectory loadCone2ToBridgeTrajectory = pathGroup.get(3);

        HashMap<String, Command> loadCone2ToBridgeEventMap = new HashMap<>();

        FollowPathWithEvents loadCone2ToBridgeWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone2ToBridgeTrajectory), 
            loadCone2ToBridgeTrajectory.getMarkers(),
            loadCone2ToBridgeEventMap);

        Command loadCone2ToBridgeCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone2ToBridgeTrajectory)
        .andThen(loadCone2ToBridgeWithEvents);

        // Open claw and retract command
        Command openClawAndRetractCommand = new AutoClawOpenCommand()
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));

        // Close claw and retract command
        Command closeClawAndRetractCommand = new AutoClawCloseCommand()
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));


        Command scoreTwoLoadAndBalanceCommand = 
        new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT)
        .andThen(openClawAndRetractCommand)
        .andThen(scorePreloadHighToLoadCone1Command)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(closeClawAndRetractCommand)
        .andThen(loadCone1ToScoreMidCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT))
        .andThen(openClawAndRetractCommand)
        .andThen(scoreMidToLoadCone2Command)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(closeClawAndRetractCommand)
        .andThen(loadCone2ToBridgeCommand)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Score Two Pieces, Load, and Balance Auto", scoreTwoLoadAndBalanceCommand);
    }
}
