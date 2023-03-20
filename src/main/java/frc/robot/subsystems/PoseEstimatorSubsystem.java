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
import frc.robot.LimelightHelpers;
import frc.robot.Robot;


public class PoseEstimatorSubsystem extends SubsystemBase {

    public final Field2d field = new Field2d();
      
    double timeStamp = Timer.getFPGATimestamp() - (Robot.LIMELIGHT_SUBSYSTEM.getTl() / 1000)
            - (Robot.LIMELIGHT_SUBSYSTEM.getCl() / 1000);

    private Matrix<N3, N1> visionStdDevs;
    private Matrix<N3, N1> stateStdDevs;

    @Override
    public void periodic() {
        Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getPose();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation();
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions();

        if (Robot.LIMELIGHT_SUBSYSTEM.getTv() == 1  && (Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX() - Robot.LIMELIGHT_SUBSYSTEM.getRobotPose().getX() < 2) &&
        Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY() - Robot.LIMELIGHT_SUBSYSTEM.getRobotPose().getY() < 2 && Robot.LIMELIGHT_SUBSYSTEM.getTa() >= 0.2 && LimelightHelpers.getFiducialID("limelight") <= 8) {
            addVisionMeasurment(Robot.LIMELIGHT_SUBSYSTEM.getRobotPose(), timeStamp);
        }
        SmartDashboard.putData(field);
        field.setRobotPose(getCurrentPose());
        updateOdometry();
        
        if(Robot.LIMELIGHT_SUBSYSTEM.getTa() <= 0.2) {
            stateStdDevs = VecBuilder.fill(0.,0, Units.degreesToRadians(0));
            visionStdDevs = VecBuilder.fill(0.,0, Units.degreesToRadians(0));
        }

        if(Robot.LIMELIGHT_SUBSYSTEM.getTa() >= 0.3) {
            stateStdDevs = VecBuilder.fill(0, 0, Units.degreesToRadians(0));
            visionStdDevs = VecBuilder.fill(0,0 ,Units.degreesToRadians(0));
        }
        
    }
    

    public void resetPosition() {
        resetPosition(Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
        Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(), Robot.LIMELIGHT_SUBSYSTEM.getRobotPose());
    }
   
    
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Robot.DRIVE_TRAIN_SUBSYSTEM.m_kinematics,
            Robot.DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getSwerveModulePositions(),
            Robot.DRIVE_TRAIN_SUBSYSTEM.getPose(),
            stateStdDevs,
            visionStdDevs);

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