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

public class TwoPointFiveConeAndBalanceCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance, AutoEnums.AutoSide autoSide) {
        String pathFile = "2.5 Cone + Balance " + autoSide.toString() + " " + autoAlliance.toString();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathFile, new PathConstraints(1.6, 0.6));

        // Path 1
        PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = pathGroup.get(0);
        
        HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();

        FollowPathWithEvents scorePreloadHighToLoadCone1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scorePreloadHighToLoadCone1Trajectory), 
            scorePreloadHighToLoadCone1Trajectory.getMarkers(),
            scorePreloadHighToLoadCone1EventMap);


        // Path 2
        PathPlannerTrajectory loadCone1ToScoreMidTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreMidEventMap = new HashMap<>();

        FollowPathWithEvents loadCone1ToScoreMidWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreMidTrajectory), 
            loadCone1ToScoreMidTrajectory.getMarkers(),
            loadCone1ToScoreMidEventMap);


        // Path 3
        PathPlannerTrajectory scoreMidToLoadCone2Trajectory = pathGroup.get(2);
        
        HashMap<String, Command> scoreMidToLoadCone2EventMap = new HashMap<>();

        FollowPathWithEvents scoreMidToLoadCone2WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoreMidToLoadCone2Trajectory), 
            scoreMidToLoadCone2Trajectory.getMarkers(),
            scoreMidToLoadCone2EventMap);


        // Path 4
        PathPlannerTrajectory loadCone2ToBridgeTrajectory = pathGroup.get(3);

        HashMap<String, Command> loadCone2ToBridgeEventMap = new HashMap<>();

        FollowPathWithEvents loadCone2ToBridgeWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone2ToBridgeTrajectory), 
            loadCone2ToBridgeTrajectory.getMarkers(),
            loadCone2ToBridgeEventMap);


        Command scoreTwoLoadAndBalanceCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scorePreloadHighToLoadCone1Trajectory)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_OPPOSITE_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scorePreloadHighToLoadCone1WithEvents)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone1ToScoreMidWithEvents)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scoreMidToLoadCone2WithEvents)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone2ToBridgeWithEvents)
        .andThen(new DrivetrainChargeStationBalancingCommand());

        return scoreTwoLoadAndBalanceCommand;
    }
}
