package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class PoseEstimatorSubsystem extends SubsystemBase {

    public final Field2d field = new Field2d();

    double timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl() / 1000)
            - (Robot.LIMELIGHT_SUBSYSTEM.getCl() / 1000);

    @Override
    public void periodic() {
        Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions();

        if (Robot.LIMELIGHT_SUBSYSTEM.getTv() == 1) {
            addVisionMeasurment(Robot.LIMELIGHT_SUBSYSTEM.getRobotPose(), timeStamp);
        }

        updateOdometry();

    }

    public void init() {

        resetPosition(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
                Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(), Robot.LIMELIGHT_SUBSYSTEM.getRobotPose());

    }

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Robot.DRIVE_TRAIN_SUBSYSTEM.m_kinematics,
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getPose(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9));

    public void resetPosition(Rotation2d rotation2d, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_poseEstimator.resetPosition(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
                Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(), Robot.LIMELIGHT_SUBSYSTEM.getRobotPose());
    }

    public void addVisionMeasurment(Pose2d robotPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(
                // m_poseEstimator.getEstimatedPosition(),
                Robot.LIMELIGHT_SUBSYSTEM.getRobotPose(),
                timeStamp);

    }

    public void updateOdometry() {
        m_poseEstimator.update(
                Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        Robot.DRIVE_TRAIN_SUBSYSTEM.m_frontLeftModule.getPosition(),
                        Robot.DRIVE_TRAIN_SUBSYSTEM.m_frontRightModule.getPosition(),
                        Robot.DRIVE_TRAIN_SUBSYSTEM.m_backLeftModule.getPosition(),
                        Robot.DRIVE_TRAIN_SUBSYSTEM.m_backRightModule.getPosition()
                });
    }

    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

}