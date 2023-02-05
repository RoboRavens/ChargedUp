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

    //var robotStartingPosition = new Pose2d(Units.feetToMeters(Robot.UPDATE_ROBOT_POSE_COMMAND.execute()),
       // Units.feetToMeters(Robot.LIMELIGHT_SUBSYSTEM.getBotpose()),
        //Rotation2d.fromDegrees(Robot.LIMELIGHT_SUBSYSTEM.getBotpose()));
    var coneTargetLocation = new Pose2d(Units.feetToMeters(5.56), Units.feetToMeters(5.14),
        Rotation2d.fromDegrees(64.80));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

   //var trajectory = TrajectoryGenerator.generateTrajectory(
       // robotStartingPosition,
   //    null, coneTargetLocation,
      //  config);

  }
}