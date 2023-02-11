package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem {

  // LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");
  int camMode = 0;

  
  public double robotposeX() {
    SmartDashboard.putNumber("botpose", botpose[0]);
    return botpose[0];
  }

  public double robotPoseY() {
    SmartDashboard.putNumber("botpose", botpose[1]);
    return botpose[1];
  }

  public double robotPoseZ() {
    SmartDashboard.putNumber("botpose", botpose[5]);
    return botpose[5];
  }

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 25.0;

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 20.0;

  // distance from the target to the floor
  double goalHeightInches = 60.0;

  public double getTx() {
    return tx.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getDistance() {
    double targetOffsetAngle_Vertical = getTy();

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToTargetInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber("DISTANCE FROM TARGET", distanceFromLimelightToTargetInches);
    return distanceFromLimelightToTargetInches;
  }

}
