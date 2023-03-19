package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class LimelightSubsystem extends SubsystemBase {

  // LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tl = table.getEntry("tl");
  NetworkTableEntry cl = table.getEntry("cl");
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

  }

  public Pose2d getRobotPose() {

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = new Rotation2d(Math.toRadians(botpose[5]));
      Pose2d position = new Pose2d(translation, rotation);
      return position;

    }
    return null;
  }

  public double getCl() {
    return cl.getDouble(0.0);
  }

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
}
