package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LimelightTrajectorySubsystem {

  Field2d m_field = new Field2d();

  public void generateTrajectory() {
    // If we give our Starting POSE2D paramater a "Getbotpose" method from limelight
    // subsystem directly, getBotpose it
    // would be sending information to our trajectory generation file all the time

    // in robotinit we set a button to a command that enables/disables the robotpose
    // function
    // We want freedom to use other pipelines to track other objects like cubes and
    // retroreflective tape but this is the better
    // way to do it

    var robotStartingPosition = new Pose2d(Units.feetToMeters(Robot.LIMELIGHT_SUBSYSTEM.robotposeX()),
        Units.feetToMeters(Robot.LIMELIGHT_SUBSYSTEM.robotPoseY()),
        Rotation2d.fromDegrees(Robot.LIMELIGHT_SUBSYSTEM.robotPoseZ()));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(4.54), Units.feetToMeters(16.23)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

    var coneTargetLocation = new Pose2d(Units.feetToMeters(5.56), Units.feetToMeters(5.14),
        Rotation2d.fromDegrees(64.80));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

     var trajectory = TrajectoryGenerator.generateTrajectory(
     robotStartingPosition,
     interiorWaypoints, 
     coneTargetLocation,
     config);

  }
}