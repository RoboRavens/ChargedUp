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
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.arm.ArmSequencedRetractionCommand;
import frc.robot.commands.arm.ArmSequencedRetractionFromReverseCommand;
import frc.robot.commands.claw.ClawOpenCommand;

public class ScoreMobilityBalanceAuto extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance) {
        String pathFile = "Score, Mobility, Balance  " + autoAlliance.toString();
        PathPlannerTrajectory path = PathPlanner.loadPath(pathFile, new PathConstraints(1.6, 0.6));
        
        HashMap<String, Command> pathEventMap = new HashMap<>();

        FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path), 
            path.getMarkers(),
            pathEventMap);

        Command pathCommand = pathWithEvents;

        Command mobilityCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_REVERSE_SETPOINT)).withTimeout(3)
        .andThen(new ClawOpenCommand().withTimeout(3))
        .andThen(new ArmSequencedRetractionFromReverseCommand()).withTimeout(3)
        .andThen(pathCommand).withTimeout(7); 

        return mobilityCommand;
    }
}
