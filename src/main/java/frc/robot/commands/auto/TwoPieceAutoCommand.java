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
import frc.util.AutoMode;

public class TwoPieceAutoCommand {
    public static AutoMode getAutoMode() {
        Command tempAutoIntakeCommand = new TempAutoIntakeCommand();
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Two Piece Auto", new PathConstraints(1.5, 0.6));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Get Game Piece", new WaitCommand(1.5).deadlineWith(tempAutoIntakeCommand));

        FollowPathWithEvents examplePathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath), 
            examplePath.getMarkers(), 
            eventMap);

        Command testCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(examplePathWithEvents)
        .andThen(() -> Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0, 0, 0)));

        return new AutoMode("Test Auto", testCommand);
    }
}
