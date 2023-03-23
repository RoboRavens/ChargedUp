package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class PoseEstimatorSubsystem extends SubsystemBase {
    public final Field2d _field = new Field2d();
    private double _timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM_ONE.getTl() / 1000)
        - (Robot.LIMELIGHT_SUBSYSTEM_ONE.getCl() / 1000);
    private double _timeStamp2 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM_TWO.getTl() / 1000)
        - (Robot.LIMELIGHT_SUBSYSTEM_TWO.getCl() / 1000);
    private Matrix<N3, N1> _stateStdDevs = VecBuilder.fill(Constants.STATE_STANDARD_DEVIATION,
        Constants.STATE_STANDARD_DEVIATION, Constants.STATE_STANDARD_DEVIATION);
    private Matrix<N3, N1> _visionStdDevs = VecBuilder.fill(Constants.STARTING_VISION_STANDARD_DEVIATION,
        Constants.STARTING_VISION_STANDARD_DEVIATION, Constants.STARTING_VISION_STANDARD_DEVIATION);

    public PoseEstimatorSubsystem() {
        SmartDashboard.putData("PoseEstimator Field", _field);
    }

    @Override
    public void periodic() {
        updateVisionMeasurementTimestamp();
        double ta = Robot.LIMELIGHT_SUBSYSTEM_ONE.getTa();
        double ta2 = Robot.LIMELIGHT_SUBSYSTEM_TWO.getTa();
        Pose2d firstLimelightPose = Robot.LIMELIGHT_SUBSYSTEM_ONE.getLimelightPoseWithOdometryRotation();
        Pose2d secondLimelightPose = Robot.LIMELIGHT_SUBSYSTEM_TWO.getLimelightPoseWithOdometryRotation();
        Pose2d robotPose = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose();
        Robot.LIMELIGHT_SUBSYSTEM_ONE.getPureLimelightRobotPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions();

        boolean hasVisionTarget = Robot.LIMELIGHT_SUBSYSTEM_ONE.hasVisionTarget() == 1;
        boolean hasVisionTarget2 = Robot.LIMELIGHT_SUBSYSTEM_TWO.hasVisionTarget() == 1;
        boolean firstLimelightIsWithinXDistance = Math.abs(robotPose.getX() - firstLimelightPose.getX()) < 2;
        boolean firstLimelightIsWithinYDistance = Math.abs(robotPose.getY() - firstLimelightPose.getY()) < 2;
        boolean secondLimelightIsWithinXDistance = Math.abs(robotPose.getX() - secondLimelightPose.getX()) < 2;
        boolean secondLimelightIsWithinYDistance = Math.abs(robotPose.getY() - secondLimelightPose.getY()) < 2;
        boolean targetAreaIsSufficient = ta >= 0.2;
        boolean targetAreaIsSufficient2 = ta2 >= 0.2;
        boolean fiducialIdIsCorrect2 = LimelightHelpers.getFiducialID("limelighttwo") <= 8;
        boolean fiducialIdIsCorrect = LimelightHelpers.getFiducialID("limelight") <= 8;

        if (hasVisionTarget && firstLimelightIsWithinXDistance && firstLimelightIsWithinYDistance
                && targetAreaIsSufficient && fiducialIdIsCorrect) {
            SmartDashboard.putBoolean("limelight has target", true);
            var vsStdDvs = PoseEstimatorSubsystem.GetVisionStdDevs(ta);
            m_poseEstimator.addVisionMeasurement(firstLimelightPose, _timeStamp, vsStdDvs);
        } else {
            SmartDashboard.putBoolean("limelight has target", false);
        }

        if (hasVisionTarget2 && secondLimelightIsWithinXDistance && secondLimelightIsWithinYDistance
                && targetAreaIsSufficient2 && fiducialIdIsCorrect2) {
            SmartDashboard.putBoolean("limelight2 has target", true);
            var vsStdDvs = PoseEstimatorSubsystem.GetVisionStdDevs(ta2);
            m_poseEstimator.addVisionMeasurement(secondLimelightPose, _timeStamp2, vsStdDvs);
        } else {
            SmartDashboard.putBoolean("limelight2 has target", false);
        }

        updateOdometry();
        _field.setRobotPose(getCurrentPose());
    }

    private static Matrix<N3, N1> GetVisionStdDevs(double ta) {
        // Scale the confidence of the vision estimate by how much ApilTag we see.
        double inverseArea = 1 - ta;
        double inverseCubed = Math.pow(inverseArea, 3);
        double clampedCube = Math.max(inverseCubed, Constants.MINIMUM_VISION_STANDARD_DEVIATION);

        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(clampedCube, clampedCube, clampedCube);

        return visionStdDevs;
    }

    public void updateVisionMeasurementTimestamp() {
        _timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM_ONE.getTl() / 1000)
            - (Robot.LIMELIGHT_SUBSYSTEM_ONE.getCl() / 1000);
    }

    public void updateVisionMeasurmentTimestampSecondLimelight() {
        _timeStamp2 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM_TWO.getTl() / 1000)
            - (Robot.LIMELIGHT_SUBSYSTEM_ONE.getCl() / 1000);
    }

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        Robot.DRIVE_TRAIN_SUBSYSTEM.m_kinematics,
        Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
        Robot.DRIVE_TRAIN_SUBSYSTEM.getPose(),
        _stateStdDevs,
        _visionStdDevs);

    public void resetOdometryPoseToLimelight() {
        var gyroBasedPose = Robot.LIMELIGHT_SUBSYSTEM_ONE.getLimelightPoseWithOdometryRotation();
       // System.out.println("resetOdometryPoseToLimelight: " + gyroBasedPose.getX() + " - " + gyroBasedPose.getY()
                + " - " + gyroBasedPose.getRotation().getDegrees());
        m_poseEstimator.resetPosition(
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            gyroBasedPose);
    }

    public void resetOdometryPoseFromSecondLimelight(Rotation2d rotation2d, SwerveModulePosition[] modulePositions,
            Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_poseEstimator.resetPosition(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.LIMELIGHT_SUBSYSTEM_TWO.getLimelightPoseWithOdometryRotation());
    }

    // public void addVisionMeasurment(Pose2d robotPose, double timestampSeconds) {
    // m_poseEstimator.addVisionMeasurement(robotPose, timestampSeconds,
    // stateStdDevs);

    // /*
    // m_poseEstimator.addVisionMeasurement(
    // // m_poseEstimator.getEstimatedPosition(),
    // Robot.LIMELIGHT_SUBSYSTEM.getPureLimelightRobotPose(),
    // timeStamp);
    // */
    // }

    public void updateOdometry() {
        m_poseEstimator.update(
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions());
    }

    public void resetPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            pose);
    }

    public void zeroGyroscope() {
        var pose = m_poseEstimator.getEstimatedPosition();
        m_poseEstimator.resetPosition(
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            new Pose2d(pose.getTranslation(), new Rotation2d()));
    }

    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
}