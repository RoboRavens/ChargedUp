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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LimelightTrajectorySubsystem {
  
  /*This code defines a LimelightTrajectorySubsystem in Java for a robot in the First Robotics Competition. 
  It creates two different trajectories, one for the left side of the robot and one for the right side, based on selected endpoints in Shuffleboard. 
  The trajectories are defined based on starting poses, interior waypoints, and selected endpoints, which are specified as Pose2d objects. 
  The code uses the TrajectoryGenerator class to generate the trajectories, which are then pushed to SmartDashboard. The interior waypoints, endpoints, 
  and starting poses are all specified in units of meters and the code converts units from feet to meters. */
  
  
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable table = networkTableInstance.getTable("Shuffleboard");
  public void generateTrajectoryLeft() {

    var robotposestart = Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();

    if (robotposestart == null) {
        return;
    }

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(16.89), Units.feetToMeters(2.23)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(8.36), Units.feetToMeters(2.23)));

    // Define the endpoints for the trajectories here.
    var endpoint1 = new Pose2d(Units.feetToMeters(6.23), Units.feetToMeters(1.44),
        Rotation2d.fromDegrees(0));
    var endpoint2 = new Pose2d(Units.feetToMeters(6.52), Units.feetToMeters(3.57),
        Rotation2d.fromDegrees(90));
    var endpoint3 = new Pose2d(Units.feetToMeters(6.39), Units.feetToMeters(5.38),
        Rotation2d.fromDegrees(180));

    var endpoint4 = new Pose2d(Units.feetToMeters(5.38), Units.feetToMeters(7.08),
        Rotation2d.fromDegrees(0));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(6));
    config.setReversed(true);

   
    

     var trajectoryleft = TrajectoryGenerator.generateTrajectory(
        robotposestart,
        interiorWaypoints,
        endpoint1,
        config);

    Field2d LEFT_FIELD = new Field2d();
    SmartDashboard.putData(LEFT_FIELD);

    // Push the trajectory to Field2d.
    LEFT_FIELD.getObject("trajectoryleft").setTrajectory(trajectoryleft);
    
  
  }
  public void generateTrajectoryRight() {
     

    var limelightRobotPose = Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();

    if (limelightRobotPose == null) {
        return;
    }

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.30), Units.feetToMeters(15.97)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(7.31), Units.feetToMeters(16.24)));

   

    // Define the endpoints for the trajectories here.
    var endpoint1 = new Pose2d(Units.feetToMeters(6.36), Units.feetToMeters(12.66),
        Rotation2d.fromDegrees(0));
    var endpoint2 = new Pose2d(Units.feetToMeters(20), Units.feetToMeters(20),
        Rotation2d.fromDegrees(0));
    var endpoint3 = new Pose2d(Units.feetToMeters(30), Units.feetToMeters(30),
        Rotation2d.fromDegrees(0));

    var endpoint4 = new Pose2d(Units.feetToMeters(30), Units.feetToMeters(30),
        Rotation2d.fromDegrees(0));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

    
  
    

    var trajectoryright = TrajectoryGenerator.generateTrajectory(
        limelightRobotPose,
        interiorWaypoints,
        endpoint1,
        config);

    Field2d m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("trajectoryright").setTrajectory(trajectoryright);
        
  }



  
  public void goToScoringPosition() {
     

    var limelightRobotPose = Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();

    if (limelightRobotPose == null) {
        return;
    }

    var interiorWaypoints = new ArrayList<Translation2d>();
    // Define the endpoints for the trajectories here.
    var endpoint1 = new Pose2d(14.76, 2.76, Rotation2d.fromDegrees(180));

    TrajectoryConfig config = new TrajectoryConfig(2,1);
    config.setReversed(true);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        limelightRobotPose,
        interiorWaypoints,
        endpoint1,
        config);

    Field2d m_field = new Field2d();
    SmartDashboard.putData(m_field);


    var driveCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(trajectory)
        .andThen(Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(trajectory));

        driveCommand.schedule();
        
  }






}
