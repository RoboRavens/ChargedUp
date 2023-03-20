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
  private NetworkTable limelightOne = NetworkTableInstance.getDefault().getTable("firstLimelight");
  private NetworkTable limelightTwo = NetworkTableInstance.getDefault().getTable("secondLimelight");
  private NetworkTableEntry tx = limelightOne.getEntry("tx");
  private NetworkTableEntry ty = limelightOne.getEntry("ty");
  private NetworkTableEntry ta = limelightOne.getEntry("ta");
  private NetworkTableEntry ts = limelightOne.getEntry("ts");
  private NetworkTableEntry tv = limelightOne.getEntry("tv");
  private NetworkTableEntry tl = limelightOne.getEntry("tl");
  private NetworkTableEntry cl = limelightOne.getEntry("cl");
  private NetworkTableEntry tx2 = limelightTwo.getEntry("tx");
  private NetworkTableEntry ty2 = limelightTwo.getEntry("ty");
  private NetworkTableEntry ta2 = limelightTwo.getEntry("ta");
  private NetworkTableEntry ts2 = limelightTwo.getEntry("ts");
  private NetworkTableEntry tv2 = limelightTwo.getEntry("tv");
  private NetworkTableEntry tl2 = limelightTwo.getEntry("tl");
  private NetworkTableEntry cl2 = limelightTwo.getEntry("cl");
  private int camMode = 0;

  public boolean isAlignedWithScoringNode() {
    // TODO: Implement this method
    return false;
  }

  public void switchToScoringPipeline() {
    // TODO: Implement this method
  }

  public void periodic() {
    Pose2d limelightOnePose = getPureLimelightRobotPose();
    Pose2d limelightTwoPose = getPureSecondLimelightRobotPose();
    if (limelightOnePose != null) {
      SmartDashboard.putNumber("LimelightOne PoseX", limelightOnePose.getX());
      SmartDashboard.putNumber("LimelightOne PoseY", limelightOnePose.getY());
      SmartDashboard.putNumber("LimelightOne Rotation", limelightOnePose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber("LimelightOne PoseX", 0);
      SmartDashboard.putNumber("LimelightOne PoseY", 0);
      SmartDashboard.putNumber("LimelightOne Rotation", 0);
    }

    if (limelightTwoPose != null) {
      SmartDashboard.putNumber("LimelightTwo PoseX", limelightTwoPose.getX());
      SmartDashboard.putNumber("LimelightTwo PoseY", limelightTwoPose.getY());
      SmartDashboard.putNumber("LimelightTwo Rotation", limelightTwoPose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber("LimelightTwo PoseX", 0);
      SmartDashboard.putNumber("LimelightTwo PoseY", 0);
      SmartDashboard.putNumber("LimelightTwo Rotation", 0);
    }
  }

  public double[] getSecondLimelightBotpose() {
    double[] botposeRight = NetworkTableInstance.getDefault().getTable("limelightRight").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);

    return botposeRight;
  }

  public double[] getLimelightBotpose() {
    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);

    return botpose;
  }

  // This gets the robot pose PURELY from the Limelight, instead of using the
  // gyroscope for rotation.
  public Pose2d getPureLimelightRobotPose() {
    double[] botpose = getLimelightBotpose();

    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = new Rotation2d(Math.toRadians(botpose[5]));
      Pose2d position = new Pose2d(translation, rotation);
      return position;
    }

    return null;
  }

  // This replaces the Limelight's rotation value with the gyroscope's value.
  public Pose2d getGyroBasedRobotPose() {
    double[] botpose = getLimelightBotpose();

    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
      Pose2d position = new Pose2d(translation, rotation);
      return position;
    }

    return null;
  }

  public Pose2d getPureSecondLimelightRobotPose() {
    double[] botposeRight = getSecondLimelightBotpose();

    if (botposeRight.length >= 6) {
      Translation2d translation = new Translation2d(botposeRight[0], botposeRight[1]);
      Rotation2d rotation = new Rotation2d(Math.toRadians(botposeRight[5]));
      Pose2d position = new Pose2d(translation, rotation);
      return position;
    }

    return null;
  }

  public Pose2d getGyroBasedRobotPoseFromSecondLimelight() {
    double[] botpose = getSecondLimelightBotpose();
    if (botpose.length >= 6) {
      Translation2d translation = new Translation2d(botpose[0], botpose[1]);
      Rotation2d rotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
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

  public double getCl2() {
    return cl2.getDouble(0.0);
  }

  public double getTl2() {
    return tl2.getDouble(0.0);
  }

  public double getTx2() {
    return tx2.getDouble(0.0);
  }

  public double getTa2() {
    return ta2.getDouble(0.0);
  }

  public double getTy2() {
    return ty2.getDouble(0.0);
  }

  public double getTv2() {
    return tv2.getDouble(0);
  }

}
