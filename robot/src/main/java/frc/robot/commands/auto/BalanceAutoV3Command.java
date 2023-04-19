package frc.robot.commands.auto;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.arm.ArmSequencedRetractionCommand;
import frc.robot.commands.arm.ArmSequencedRetractionFromReverseCommand;
import frc.robot.commands.claw.ClawOpenCommand;

public class BalanceAutoV3Command extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance) {
        String pathFile = "Balance V3 " + autoAlliance.toString();
        PathPlannerTrajectory path = PathPlanner.loadPath(pathFile, new PathConstraints(1.6, 0.6));
        
        HashMap<String, Command> pathEventMap = new HashMap<>();

        FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path), 
            path.getMarkers(),
            pathEventMap);

        Command pathCommand = pathWithEvents;

        Command balanceAutoCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_REVERSE_SETPOINT).withTimeout(2.25))
        .andThen(new ClawOpenCommand().withTimeout(.5))
        .andThen(new ArmSequencedRetractionFromReverseCommand().withTimeout(2))
        .andThen(new DriveUntilTiltedCommand().withTimeout(3))
        .andThen(new DriveUntilLevelCommand().withTimeout(4))
        .andThen(new WaitCommand(.5))
        .andThen(new DriveBackwardUntilLevelCommand())
        .andThen(new InstantCommand(() -> Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition()));
        //.andThen(pathCommand).withTimeout(7); 

        return balanceAutoCommand;
    }
}
