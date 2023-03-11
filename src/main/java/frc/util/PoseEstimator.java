package frc.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using a
 * {@link SwerveDrivePoseEstimator}, and robot pose sources.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator extends SubsystemBase {

    public final Field2d field = new Field2d();
    public final Pose2d robotPose = Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();
    public final Pose2d swervePose = Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
    public final Rotation2d rotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Robot.DRIVE_TRAIN_SUBSYSTEM.m_kinematics,
            rotation,
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            swervePose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
            
           
 
    public void addVisionMeasurment(Pose2d robotPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(
            m_poseEstimator.getEstimatedPosition(),
        Timer.getFPGATimestamp() - 0.3);

    }

    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

}
