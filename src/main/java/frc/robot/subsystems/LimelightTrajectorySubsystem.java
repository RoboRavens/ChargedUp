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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    public Pose2d selectEndpoint() {
        SendableChooser<Pose2d> poseChooser = new SendableChooser<>();
        Pose2d row1BlueAlliance = new Pose2d(1.9 + Constants.ROBOT_SCORING_OFFSET, 0.46, Rotation2d.fromDegrees(0));
        Pose2d row2BlueAlliance = new Pose2d(1.87 + Constants.ROBOT_SCORING_OFFSET, 1.09, Rotation2d.fromDegrees(0));
        Pose2d row3BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 1.62, Rotation2d.fromDegrees(0));
        Pose2d row4BlueAlliance = new Pose2d(1.89 + Constants.ROBOT_SCORING_OFFSET, 2.19, Rotation2d.fromDegrees(0));
        Pose2d row5BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 2.72, Rotation2d.fromDegrees(0));
        Pose2d row6BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 3.28, Rotation2d.fromDegrees(0));
        Pose2d row7BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 3.84, Rotation2d.fromDegrees(0));
        Pose2d row8BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 4.39, Rotation2d.fromDegrees(0));
        Pose2d row9BlueAlliance = new Pose2d(1.91 + Constants.ROBOT_SCORING_OFFSET, 5.02, Rotation2d.fromDegrees(0));
        Pose2d row1RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 5.02, Rotation2d.fromDegrees(0));
        Pose2d row2RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 4.39, Rotation2d.fromDegrees(0));
        Pose2d row3RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 3.85, Rotation2d.fromDegrees(0));
        Pose2d row4RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 3.29, Rotation2d.fromDegrees(0));
        Pose2d row5RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 2.73, Rotation2d.fromDegrees(0));
        Pose2d row6RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 2.16, Rotation2d.fromDegrees(0));
        Pose2d row7RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 1.62, Rotation2d.fromDegrees(0));
        Pose2d row8RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 1.06, Rotation2d.fromDegrees(0));
        Pose2d row9RedAlliance = new Pose2d(14.61 - Constants.ROBOT_SCORING_OFFSET, 0.5, Rotation2d.fromDegrees(0));
        poseChooser.setDefaultOption("Initial Pose", row1BlueAlliance);
        poseChooser.addOption("ROW 1 CONE BLUE ALLIANCE", row1BlueAlliance);
        poseChooser.addOption("ROW 2 CUBE BLUE ALLIANCE", row2BlueAlliance);
        poseChooser.addOption("ROW 3 CONE BLUE ALLIANCE", row3BlueAlliance);
        poseChooser.addOption("ROW 4 CONE BLUE ALLIANCE", row4BlueAlliance);
        poseChooser.addOption("ROW 5 CUBE BLUE ALLIANCE", row5BlueAlliance);
        poseChooser.addOption("ROW 6 CONE BLUE ALLIANCE", row6BlueAlliance);
        poseChooser.addOption("ROW 7 CONE BLUE ALLIANCE", row7BlueAlliance);
        poseChooser.addOption("ROW 8 CUBE BLUE ALLIANCE", row8BlueAlliance);
        poseChooser.addOption("ROW 9 CONE BLUE ALLIANCE", row9BlueAlliance);
        poseChooser.addOption("ROW 1 CONE RED ALLIANCE", row1RedAlliance);
        poseChooser.addOption("ROW 2 CUBE RED ALLIANCE", row2RedAlliance);
        poseChooser.addOption("ROW 3 CONE RED ALLIANCE", row3RedAlliance);
        poseChooser.addOption("ROW 4 CONE RED ALLIANCE", row4RedAlliance);
        poseChooser.addOption("ROW 5 CUBE RED ALLIANCE", row5RedAlliance);
        poseChooser.addOption("ROW 6 CONE RED ALLIANCE", row6RedAlliance);
        poseChooser.addOption("ROW 7 CONE RED ALLIANCE", row7RedAlliance);
        poseChooser.addOption("ROW 8 CUBE RED ALLIANCE", row8RedAlliance);
        poseChooser.addOption("ROW 9 CONE RED ALLIANCE", row9RedAlliance);
        SmartDashboard.putData("Pose Selector", poseChooser);
        Pose2d selectedPose = poseChooser.getSelected();
        return selectedPose;
    }

    public void periodic() {

        goToScoringPosition();
        DASHBOARD_Field2d.setRobotPose(Robot.DRIVE_TRAIN_SUBSYSTEM.getPose());
    }

    public Trajectory goToScoringPosition() {

        var robotPose = Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();

        if (robotPose == null) {
            return null;
        }

        var interiorWaypoints = new ArrayList<Translation2d>();
        // Define the endpoints for the trajectories here.

        Pose2d selectedPose = selectEndpoint();

        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        // config.setReversed(true);
        try {
            var TRAJECTORY1 = TrajectoryGenerator.generateTrajectory(
                    robotPose,
                    interiorWaypoints,
                    selectedPose,
                    config);
            DASHBOARD_Field2d.getObject("trajectory").setTrajectory(TRAJECTORY1);
            return TRAJECTORY1;
        } catch (Exception e) {
            System.out.println(e.getStackTrace());
        }

        return null;
    }

    public void driveTrajectory() {
        Trajectory Trajectory = goToScoringPosition();
        if (Trajectory == null) {
            return;
        }

        var driveCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory)
                .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(Trajectory));

        driveCommand.schedule();
    }
}
