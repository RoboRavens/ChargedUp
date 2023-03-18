package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.util.AutoMode;

public class PreloadExitAllianceAndBalanceAutoCommand {
    public static AutoMode getAutoMode() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Score Preload, Cross Alliance Line and Balance", new PathConstraints(1, 0.4));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score Preload", new WaitCommand(1).deadlineWith(new ScorePieceCommand()));
        // Unhandled exception: java.lang.IllegalArgumentException: Events that are triggered during path following cannot require the drive subsystem
        // eventMap.put("Balance Charge Station", new ChargeStationBalancingCommand());

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(), 
            eventMap);

        Command testCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(examplePathWithEvents)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Test Auto", testCommand);
    }
}
