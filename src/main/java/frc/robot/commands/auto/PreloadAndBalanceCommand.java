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
import frc.robot.commands.TempAutoIntakeCommand;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.util.AutoMode;

public class PreloadAndBalanceCommand {
    public static AutoMode getAutoMode() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Preload and Balance", new PathConstraints(1, 0.4));

        HashMap<String, Command> eventMap = new HashMap<>();

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(), 
            eventMap);

        Command preloadAndBalanceCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(new TempAutoIntakeCommand()) // TODO: Replace with ScoreGamePieceCommand
        .andThen(examplePathWithEvents)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Preload and Balance", preloadAndBalanceCommand);
    }
}
