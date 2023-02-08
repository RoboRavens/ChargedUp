package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class LimelightTrajectorySubsystem {
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable table = networkTableInstance.getTable("Shuffleboard");
  private NetworkTableEntry targetSelection = table.getEntry("Target Selection");

  public void generateTrajectory() {

    var robotposestart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
        Rotation2d.fromDegrees(-180));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(3.90), Units.feetToMeters(4.71)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(3.88), Units.feetToMeters(4.76)));

    // Define the endpoints for the trajectories here.
    var endpoint1 = new Pose2d(Units.feetToMeters(10), Units.feetToMeters(10),
        Rotation2d.fromDegrees(0));
    var endpoint2 = new Pose2d(Units.feetToMeters(20), Units.feetToMeters(20),
        Rotation2d.fromDegrees(90));
    var endpoint3 = new Pose2d(Units.feetToMeters(30), Units.feetToMeters(30),
        Rotation2d.fromDegrees(180));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

    // Get the selected endpoint from Shuffleboard.
    int selectedEndpoint = (int) targetSelection.getDouble(0.0);

    Pose2d selectedTarget = new Pose2d();
    switch (selectedEndpoint) {
      case 0:
        selectedTarget = endpoint1;
        break;
      case 1:
        selectedTarget = endpoint2;
        break;
      case 2:
        selectedTarget = endpoint3;
        break;
    }

    var trajectory = TrajectoryGenerator.generateTrajectory(
        robotposestart,
        interiorWaypoints,
        selectedTarget,
        config);
  }
}
