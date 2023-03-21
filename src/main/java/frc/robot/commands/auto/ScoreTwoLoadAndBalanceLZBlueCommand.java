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
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.claw.AutoClawCloseCommand;
import frc.robot.commands.claw.AutoClawOpenCommand;
import frc.robot.commands.drivetrain.DrivetrainChargeStationBalancingCommand;
import frc.util.CommandSupplier;

public class ScoreTwoLoadAndBalanceLZBlueCommand {
    public static Command getAutoMode() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2.5 Cone + Balance LZ Blue", new PathConstraints(1.6, 0.6));

        // Path 1
        PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = pathGroup.get(0);
        
        HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();

        FollowPathWithEvents scorePreloadHighToLoadCone1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scorePreloadHighToLoadCone1Trajectory), 
            scorePreloadHighToLoadCone1Trajectory.getMarkers(),
            scorePreloadHighToLoadCone1EventMap);

        Command scorePreloadHighToLoadCone1PathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scorePreloadHighToLoadCone1Trajectory)
        .andThen(scorePreloadHighToLoadCone1WithEvents);


        // Path 2
        PathPlannerTrajectory loadCone1ToScoreMidTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreMidEventMap = new HashMap<>();
        loadCone1ToScoreMidEventMap.put("Extend To Score Mid", new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT));

        FollowPathWithEvents loadCone1ToScoreMidWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreMidTrajectory), 
            loadCone1ToScoreMidTrajectory.getMarkers(),
            loadCone1ToScoreMidEventMap);

        Command loadCone1ToScoreMidPathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone1ToScoreMidTrajectory)
        .andThen(loadCone1ToScoreMidWithEvents);


        // Path 3
        PathPlannerTrajectory scoreMidToLoadCone2Trajectory = pathGroup.get(2);
        
        HashMap<String, Command> scoreMidToLoadCone2EventMap = new HashMap<>();

        FollowPathWithEvents scoreMidToLoadCone2WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoreMidToLoadCone2Trajectory), 
            scoreMidToLoadCone2Trajectory.getMarkers(),
            scoreMidToLoadCone2EventMap);

        Command scoreMidToLoadCone2PathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoreMidToLoadCone2Trajectory)
        .andThen(scoreMidToLoadCone2WithEvents);


        // Path 4
        PathPlannerTrajectory loadCone2ToBridgeTrajectory = pathGroup.get(3);

        HashMap<String, Command> loadCone2ToBridgeEventMap = new HashMap<>();

        FollowPathWithEvents loadCone2ToBridgeWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone2ToBridgeTrajectory), 
            loadCone2ToBridgeTrajectory.getMarkers(),
            loadCone2ToBridgeEventMap);

        Command loadCone2ToBridgePathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone2ToBridgeTrajectory)
        .andThen(loadCone2ToBridgeWithEvents);


        Command scoreTwoLoadAndBalanceCommand = 
        new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT)
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scorePreloadHighToLoadCone1PathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone1ToScoreMidPathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scoreMidToLoadCone2PathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone2ToBridgePathCommand)
        .andThen(new DrivetrainChargeStationBalancingCommand());

        return scoreTwoLoadAndBalanceCommand;
    }
}
