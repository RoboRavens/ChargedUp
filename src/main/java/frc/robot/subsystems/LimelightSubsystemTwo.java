package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimelightSubsystemTwo extends SubsystemBase {
    private NetworkTable limelightTwo = NetworkTableInstance.getDefault().getTable("secondLimelight");
    private NetworkTableEntry tx = limelightTwo.getEntry("tx");
    private NetworkTableEntry ty = limelightTwo.getEntry("ty");
    private NetworkTableEntry ta = limelightTwo.getEntry("ta");
    private NetworkTableEntry ts = limelightTwo.getEntry("ts");
    private NetworkTableEntry tv = limelightTwo.getEntry("tv");
    private NetworkTableEntry tl = limelightTwo.getEntry("tl");
    private NetworkTableEntry cl = limelightTwo.getEntry("cl");

    public void periodic() {
       Pose2d robotPose = getPureLimelightRobotPose();
        if (robotPose != null) {
            SmartDashboard.putNumber("LimelightTwo PoseX", robotPose.getX());
            SmartDashboard.putNumber("LimelightTwo PoseY", robotPose.getY());
            SmartDashboard.putNumber("LimelightTwo Rotation", robotPose.getRotation().getDegrees());
          } else {
            SmartDashboard.putNumber("LimelightTwo PoseX", 0);
            SmartDashboard.putNumber("LimelightTwo PoseY", 0);
            SmartDashboard.putNumber("LimelightTwo Rotation", 0);
          }
    }

    public double[] getLimelightRobotPose() {
        double[] robotPoseTwo = NetworkTableInstance.getDefault().getTable("limelightTwo").getEntry("botpose_wpiblue")
                .getDoubleArray(new double[6]);

        return robotPoseTwo;
    }

    public Pose2d getPureLimelightRobotPose() {
        double[] robotPose = getLimelightRobotPose();
        if (robotPose.length >= 6) {
            Translation2d translation = new Translation2d(robotPose[0], robotPose[1]);
            Rotation2d rotation = new Rotation2d(Math.toRadians(robotPose[5]));
            Pose2d position = new Pose2d(translation, rotation);
            return position;
        }

        return null;
    }

    public Pose2d getGyroBasedRobotPose() {
      double[] robotPose = getLimelightRobotPose();
        if (robotPose.length >= 6) {
            Translation2d translation = new Translation2d(robotPose[0], robotPose[1]);
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
}
