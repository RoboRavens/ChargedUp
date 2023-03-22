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

public class TwoConeAndBalanceAutoCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance, AutoEnums.AutoSide autoSide) {
        String pathFile = "2 Cone + Balance " + autoSide.toString() + " " + autoAlliance.toString();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathFile, new PathConstraints(1.6, 0.6));

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
        PathPlannerTrajectory loadCone1ToScoreHighTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreHighEventMap = new HashMap<>();

        FollowPathWithEvents loadCone1ToScoreHighWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreHighTrajectory), 
            loadCone1ToScoreHighTrajectory.getMarkers(),
            loadCone1ToScoreHighEventMap);

        Command loadCone1ToScoreHighPathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone1ToScoreHighTrajectory)
        .andThen(loadCone1ToScoreHighWithEvents);


        // Path 3
        PathPlannerTrajectory scoreHighToBridgeTrajectory = pathGroup.get(2);
        
        HashMap<String, Command> scoreHighToBridgeEventMap = new HashMap<>();

        FollowPathWithEvents scoreHighToBridgeWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoreHighToBridgeTrajectory), 
            scoreHighToBridgeTrajectory.getMarkers(),
            scoreHighToBridgeEventMap);

        Command scoreHighToBridgePathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoreHighToBridgeTrajectory)
        .andThen(scoreHighToBridgeWithEvents);


        Command scoreTwoAndBalanceCommand = 
        new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_OPPOSITE_SETPOINT)
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scorePreloadHighToLoadCone1PathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone1ToScoreHighPathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scoreHighToBridgePathCommand)
        .andThen(new DrivetrainChargeStationBalancingCommand());

        return scoreTwoAndBalanceCommand;
    }
}
