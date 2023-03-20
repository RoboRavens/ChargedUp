package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;


public class PoseEstimatorSubsystem extends SubsystemBase {
    public final Field2d field = new Field2d();
    double timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl() / 1000) - (Robot.LIMELIGHT_SUBSYSTEM.getCl() / 1000);
    double timeStamp2 = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl2() / 1000) - (Robot.LIMELIGHT_SUBSYSTEM.getCl2() / 1000);
    private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(Constants.STATE_STANDARD_DEVIATION, Constants.STATE_STANDARD_DEVIATION, Constants.STATE_STANDARD_DEVIATION);
    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(Constants.STARTING_VISION_STANDARD_DEVIATION, Constants.STARTING_VISION_STANDARD_DEVIATION, Constants.STARTING_VISION_STANDARD_DEVIATION);
    
    @Override
    public void periodic() {
        updateVisionMeasurementTimestamp();
        double ta = Robot.LIMELIGHT_SUBSYSTEM.getTa();
        double ta2 = Robot.LIMELIGHT_SUBSYSTEM.getTa2();
        Pose2d firstLimelightPose = Robot.LIMELIGHT_SUBSYSTEM.getGyroBasedRobotPose();
        Pose2d secondLimelightPose =  Robot.LIMELIGHT_SUBSYSTEM.getGyroBasedRobotPoseFromSecondLimelight();
        Pose2d robotPose = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose();
        Robot.LIMELIGHT_SUBSYSTEM.getPureLimelightRobotPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions();

        boolean hasTarget = Robot.LIMELIGHT_SUBSYSTEM.getTv() == 1;
        boolean hasTarget2 = Robot.LIMELIGHT_SUBSYSTEM.getTv2() == 1;
        boolean firstLimelightIsWithinXDistance = Math.abs(robotPose.getX() - firstLimelightPose.getX()) < 2;
        boolean firstLimelightIsWithinYDistance = Math.abs(robotPose.getY() - firstLimelightPose.getY()) < 2;
        boolean secondLimelightIsWithinXDistance = Math.abs(robotPose.getX() - firstLimelightPose.getX()) < 2;
        boolean secondLimelightIsWithinYDistance = Math.abs(robotPose.getY() - firstLimelightPose.getY()) < 2;
        boolean targetAreaIsSufficient = ta >= 0.2;
        boolean targetAreaIsSufficient2 = ta2 >= 0.2;
        boolean fiducialIdIsCorrect = LimelightHelpers.getFiducialID("secondlimelight") <= 8;
        boolean fiducialIdIsCorrect2 = LimelightHelpers.getFiducialID("firstLimelight") <= 8;

        // Scale the confidence of the vision estimate by how much ApilTag we see.
        double inverseArea = 1 - ta;
        double inverseCubed = Math.pow(inverseArea, 3);
        double clampedCube = Math.max(inverseCubed, Constants.MINIMUM_VISION_STANDARD_DEVIATION);

        visionStdDevs = VecBuilder.fill(clampedCube, clampedCube, clampedCube);

        if (hasTarget && firstLimelightIsWithinXDistance && firstLimelightIsWithinYDistance && targetAreaIsSufficient && fiducialIdIsCorrect) {
            m_poseEstimator.addVisionMeasurement(firstLimelightPose, timeStamp, visionStdDevs);
        }


        if (hasTarget2 && secondLimelightIsWithinXDistance && secondLimelightIsWithinYDistance && targetAreaIsSufficient2 && fiducialIdIsCorrect2) {
            m_poseEstimator.addVisionMeasurement(secondLimelightPose, timeStamp2, visionStdDevs);
        }

        SmartDashboard.putData(field);
        field.setRobotPose(getCurrentPose());
        updateOdometry();
    }

    public void updateVisionMeasurementTimestamp() {
        timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl() / 1000) - (Robot.LIMELIGHT_SUBSYSTEM.getCl() / 1000);
    }

    public void updateVisionMeasurmentTimestampSecondLimelight() {
        timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl2() / 1000) - (Robot.LIMELIGHT_SUBSYSTEM.getCl2() / 1000);
    }
    
    public void resetOdometryPoseToLimelight() {
        resetOdometryPose(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(), Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(), Robot.LIMELIGHT_SUBSYSTEM.getGyroBasedRobotPose());
    }
   
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Robot.DRIVE_TRAIN_SUBSYSTEM.m_kinematics,
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getPose(),
            stateStdDevs,
            visionStdDevs);

    public void resetOdometryPose(Rotation2d rotation2d, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_poseEstimator.resetPosition(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(), Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(), Robot.LIMELIGHT_SUBSYSTEM.getGyroBasedRobotPose());
    }

    // public void addVisionMeasurment(Pose2d robotPose, double timestampSeconds) {
    //     m_poseEstimator.addVisionMeasurement(robotPose, timestampSeconds, stateStdDevs);

    //     /*
    //     m_poseEstimator.addVisionMeasurement(
    //             // m_poseEstimator.getEstimatedPosition(),
    //             Robot.LIMELIGHT_SUBSYSTEM.getPureLimelightRobotPose(),
    //             timeStamp);
    //     */
    // }

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