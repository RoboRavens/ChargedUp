package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.util.AutoMode;

public class TestAutoCommand {
    public static AutoMode getAutoMode() {
        Trajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(1.5, 0.6));
        // Trajectory exampleTrajectory =
        // TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(new Translation2d(0.5, 0.5)),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(1, 0, new Rotation2d(0)),
        //     // Pass config
        //     Robot.DRIVE_TRAIN_SUBSYSTEM.GetTrajectoryConfig());

        Command testCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(examplePath)
        .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(examplePath));

        return new AutoMode("Test Auto", testCommand);
    }
}
