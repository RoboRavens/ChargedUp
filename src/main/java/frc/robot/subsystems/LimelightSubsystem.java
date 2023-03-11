package frc.robot.subsystems;

import javax.swing.text.Position;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  // LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tl = table.getEntry("tl");
  int camMode = 0;
  
  public boolean isAlignedWithScoringNode() {
     // TODO: Implement this method
     return false;
  }

  public void switchToScoringPipeline() {
    // TODO: Implement this method
  }

  public void periodic() {
    Pose2d pose = getRobotPose();
    if (pose != null) {
      SmartDashboard.putNumber("PoseX", pose.getX());
      SmartDashboard.putNumber("PoseY", pose.getY());
      SmartDashboard.putNumber("Rotation", pose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber("PoseX", 0);
      SmartDashboard.putNumber("PoseY", 0);
      SmartDashboard.putNumber("Rotation", 0);
    }

    if ((pose.getX() > 0 && pose.getY() > 0) && getTa() >= 0.2) {
      Robot.DRIVE_TRAIN_SUBSYSTEM.resetOdometry(pose);
    }

  }

  public Pose2d getRobotPose() {

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
    if (botpose.length == 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = new Rotation2d(Math.toRadians(botpose[5]));
      Pose2d position = new Pose2d(translation, rotation);
      return position;

    }
    return null;
  }



  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 25.0;

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 20.0;

  // distance from the target to the floor
  double goalHeightInches = 60.0;

  public double getTl() {
    return tl.getDouble(0.0);
  }

  public double getTx() {
    return tx.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getTv() {
    return tv.getDouble(0);
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
