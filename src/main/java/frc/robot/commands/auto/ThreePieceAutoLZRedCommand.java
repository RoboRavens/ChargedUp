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
// import frc.util.AutoMode;
import frc.util.CommandSupplier;

public class ThreePieceAutoLZRedCommand {
    public static Command getAutoMode() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 Cone LZ Red", new PathConstraints(1.6, 0.6));

        // Path 1
        PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = pathGroup.get(0);
        
        HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();
        scorePreloadHighToLoadCone1EventMap.put("Extend To Ground 1", new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));

        FollowPathWithEvents scorePreloadHighToLoadCone1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scorePreloadHighToLoadCone1Trajectory), 
            scorePreloadHighToLoadCone1Trajectory.getMarkers(),
            scorePreloadHighToLoadCone1EventMap);

        Command scorePreloadHighToLoadCone1PathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scorePreloadHighToLoadCone1Trajectory)
        .andThen(scorePreloadHighToLoadCone1WithEvents);


        // Path 2
        PathPlannerTrajectory loadCone1ToScoreHighTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreHighEventMap = new HashMap<>();
        loadCone1ToScoreHighEventMap.put("Extend To Score High 1", new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT));

        FollowPathWithEvents loadCone1ToScoreHighWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreHighTrajectory), 
            loadCone1ToScoreHighTrajectory.getMarkers(),
            loadCone1ToScoreHighEventMap);

        Command loadCone1ToScoreHighPathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone1ToScoreHighTrajectory)
        .andThen(loadCone1ToScoreHighWithEvents);


        // Path 3
        PathPlannerTrajectory scoreHighToLoadCone2Trajectory = pathGroup.get(2);
        
        HashMap<String, Command> scoreHighToLoadCone2EventMap = new HashMap<>();
        scoreHighToLoadCone2EventMap.put("Extend To Ground 2", new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));

        FollowPathWithEvents scoreHighToLoadCone2WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoreHighToLoadCone2Trajectory), 
            scoreHighToLoadCone2Trajectory.getMarkers(),
            scoreHighToLoadCone2EventMap);

        Command scoreHighToLoadCone2PathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoreHighToLoadCone2Trajectory)
        .andThen(scoreHighToLoadCone2WithEvents);


        // Path 4
        PathPlannerTrajectory loadCone2ToScoreMidTrajectory = pathGroup.get(3);

        HashMap<String, Command> loadCone2ToScoreMidEventMap = new HashMap<>();
        loadCone2ToScoreMidEventMap.put("Extend Arm To Mid 1", new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT));

        FollowPathWithEvents loadCone2ToScoreMidWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone2ToScoreMidTrajectory), 
            loadCone2ToScoreMidTrajectory.getMarkers(),
            loadCone2ToScoreMidEventMap);

        Command loadCone2ToScoreMidPathCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(loadCone2ToScoreMidTrajectory)
        .andThen(loadCone2ToScoreMidWithEvents);

        Command scoreTwoLoadAndBalanceCommand = 
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
        .andThen(scoreHighToLoadCone2PathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT)) 
        .andThen(new AutoClawCloseCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(loadCone2ToScoreMidPathCommand)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));

        return scoreTwoLoadAndBalanceCommand;
    }
}
