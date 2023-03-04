package frc.robot.subsystems;

import java.util.ArrayList;

import javax.swing.text.Position;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimelightTrajectorySubsystem extends SubsystemBase {

    /*
     * This code defines a LimelightTrajectorySubsystem in Java for a robot in the
     * First Robotics Competition.
     * It creates two different trajectories, one for the left side of the robot and
     * one for the right side, based on selected endpoints in Shuffleboard.
     * The trajectories are defined based on starting poses, interior waypoints, and
     * selected endpoints, which are specified as Pose2d objects.
     * The code uses the TrajectoryGenerator class to generate the trajectories,
     * which are then pushed to SmartDashboard. The interior waypoints, endpoints,
     * and starting poses are all specified in units of meters and the code converts
     * units from feet to meters.
     */

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = networkTableInstance.getTable("Shuffleboard");
    Field2d DASHBOARD_Field2d = new Field2d();

    Pose2d pose = Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();

    public LimelightTrajectorySubsystem() {

        SmartDashboard.putData("trajectory", DASHBOARD_Field2d);

    }

    public void periodic() {

        goToScoringPosition();

    }

    public Trajectory goToScoringPosition() {

        var limelightRobotPose = Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();

        if (limelightRobotPose == null) {
            return null;
        }

        var interiorWaypoints = new ArrayList<Translation2d>();
        // Define the endpoints for the trajectories here.
        var endpoint1 = new Pose2d(14.76, 2.76, Rotation2d.fromDegrees(0));

        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setReversed(true);

        var TRAJECTORY1 = TrajectoryGenerator.generateTrajectory(
                limelightRobotPose,
                interiorWaypoints,
                endpoint1,
                config);
        return TRAJECTORY1;

    }

    public void driveTrajectory() {
        Trajectory Trajectory = goToScoringPosition();
        var driveCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory).andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(Trajectory));

        driveCommand.schedule();

            DASHBOARD_Field2d.getObject("trajectory").setTrajectory(Trajectory);

    }
}
