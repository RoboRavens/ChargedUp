package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.arm.ArmScoreConeHighStagingSetpointCommand;
import frc.robot.commands.arm.ArmSequencedRetractionCommand;
import frc.robot.commands.arm.ArmSequencedRetractionFromReverseCommand;
import frc.robot.commands.claw.AutoClawCloseCommand;
import frc.robot.commands.claw.AutoClawOpenCommand;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;

public class TwoConeAndMobilityAutoCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance, AutoEnums.AutoSide autoSide) {
        String pathFile = "2 Cone LZ " + autoAlliance.toString();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathFile, new PathConstraints(1.6, 0.6));

        // Path 1
        PathPlannerTrajectory scorePreloadHighToLoadCone1Trajectory = pathGroup.get(0);
        
        HashMap<String, Command> scorePreloadHighToLoadCone1EventMap = new HashMap<>();

        FollowPathWithEvents scorePreloadHighToLoadCone1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scorePreloadHighToLoadCone1Trajectory), 
            scorePreloadHighToLoadCone1Trajectory.getMarkers(),
            scorePreloadHighToLoadCone1EventMap);


        // Path 2
        PathPlannerTrajectory loadCone1ToScoreHighTrajectory = pathGroup.get(1);

        HashMap<String, Command> loadCone1ToScoreHighEventMap = new HashMap<>();

        FollowPathWithEvents loadCone1ToScoreHighWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(loadCone1ToScoreHighTrajectory), 
            loadCone1ToScoreHighTrajectory.getMarkers(),
            loadCone1ToScoreHighEventMap);

        SmartDashboard.putBoolean("Finish claw open", false);
        SmartDashboard.putBoolean("Finished retract", false);
        SmartDashboard.putBoolean("Finished path 1", false);


        Command scoreTwoAndBalanceCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scorePreloadHighToLoadCone1Trajectory)
        .andThen(new ArmScoreConeHighStagingSetpointCommand().withTimeout(3.5))
        .andThen(new ClawOpenCommand().withTimeout(3))
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Finish claw open", true)))
        .andThen(new ArmSequencedRetractionFromReverseCommand().withTimeout(3))
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Finished retract", true)))
        .andThen(scorePreloadHighToLoadCone1WithEvents.alongWith(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT).withTimeout(3)))
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Finished path 1", true)))
        .andThen(new ClawCloseCommand().withTimeout(3))
        .andThen(new ArmSequencedRetractionCommand().withTimeout(3))
        .andThen(loadCone1ToScoreHighWithEvents)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT).withTimeout(3.5))
        .andThen(new ClawOpenCommand().withTimeout(3))
        .andThen(new ArmSequencedRetractionCommand().withTimeout(3));
        
        return scoreTwoAndBalanceCommand;
    }
}
