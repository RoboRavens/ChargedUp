package frc.robot.shuffleboard;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDiagnosticsShuffleboard {
    private final GenericEntry _nteMaxVelocity;
    private final GenericEntry _nteMaxAngular;
  
    private final GenericEntry _odometryXKinematics;
    private final GenericEntry _odometryYKinematics;
    private final GenericEntry _odometryAngleKinematics;
    private final GenericEntry _frontLeftKinematics;
    private final GenericEntry _frontRightKinematics;
    private final GenericEntry _backLeftKinematics;
    private final GenericEntry _backRightKinematics;
    private final GenericEntry _frontLeftKinematicsVel;
  
    private final GenericEntry _odometryXHardware;
    private final GenericEntry _odometryYHardware;
    private final GenericEntry _odometryAngleHardware;
    private final GenericEntry _frontLeftHardware;
    private final GenericEntry _frontRightHardware;
    private final GenericEntry _backLeftHardware;
    private final GenericEntry _backRightHardware;
    private final GenericEntry _frontLeftHardwareVel;

    public DrivetrainDiagnosticsShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        _nteMaxVelocity  = tab.add("Max Velocity", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        _nteMaxAngular  = tab.add("Max Angular Vel", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    
        _nteMaxVelocity.setDouble(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        _nteMaxAngular.setDouble(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    
        _odometryXKinematics = tab.add("X Kinematics", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
        _odometryYKinematics = tab.add("Y Kinematics", 0.0).withPosition(8, 1).withSize(1, 1).getEntry();
        _odometryAngleKinematics = tab.add("Angle Kinematics", 0.0).withPosition(8, 2).withSize(1, 1).getEntry();
        _frontLeftKinematics = tab.add("FL Kinematics A", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        _frontRightKinematics  = tab.add("FR Kinematics A", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        _backLeftKinematics  = tab.add("BL Kinematics A", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
        _backRightKinematics  = tab.add("BR Kinematics A", 0.0).withPosition(4, 3).withSize(1, 1).getEntry();
        _frontLeftKinematicsVel = tab.add("FL Kinematics V", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();
    
        _odometryXHardware = tab.add("X Hardware", 0.0).withPosition(9, 0).withSize(1, 1).getEntry();
        _odometryYHardware = tab.add("Y Hardware", 0.0).withPosition(9, 1).withSize(1, 1).getEntry();
        _odometryAngleHardware = tab.add("Angle Hardware", 0.0).withPosition(9, 2).withSize(1, 1).getEntry();
        _frontLeftHardware = tab.add("FL Hardware A", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
        _frontRightHardware  = tab.add("FR Hardware A", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        _backLeftHardware  = tab.add("BL Hardware A", 0.0).withPosition(5, 2).withSize(1, 1).getEntry();
        _backRightHardware  = tab.add("BR Hardware A", 0.0).withPosition(5, 3).withSize(1, 1).getEntry();
        _frontLeftHardwareVel = tab.add("FL Hardware V", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();
    }

    public void updateKinematics(SwerveDriveOdometry odometry, SwerveModuleState[] states) {
        _odometryXKinematics.setDouble(odometry.getPoseMeters().getX());
        _odometryYKinematics.setDouble(odometry.getPoseMeters().getY());
        _odometryAngleKinematics.setDouble(odometry.getPoseMeters().getRotation().getDegrees());
        _frontLeftKinematics.setDouble(states[0].angle.getDegrees());
        _frontLeftKinematicsVel.setDouble(states[0].speedMetersPerSecond);
        _frontRightKinematics.setDouble(states[1].angle.getDegrees());
        _backLeftKinematics.setDouble(states[2].angle.getDegrees());
        _backRightKinematics.setDouble(states[3].angle.getDegrees());
    }

    public void updateHardware(SwerveDriveOdometry odometry, SwerveModuleState[] states) {
        _odometryXHardware.setDouble(odometry.getPoseMeters().getX());
        _odometryYHardware.setDouble(odometry.getPoseMeters().getY());
        _odometryAngleHardware.setDouble(odometry.getPoseMeters().getRotation().getDegrees());
        _frontLeftHardware.setDouble(states[0].angle.getDegrees());
        _frontLeftHardwareVel.setDouble(states[0].speedMetersPerSecond);
        _frontRightHardware.setDouble(states[1].angle.getDegrees());
        _backLeftHardware.setDouble(states[2].angle.getDegrees());
        _backRightHardware.setDouble(states[3].angle.getDegrees());
    }
}