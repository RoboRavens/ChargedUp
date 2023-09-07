// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
// import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
// import com.swervedrivespecialties.swervelib.Mk4SwerveModuleBuilder;
// import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.RavenSwerveControllerCommand;
import frc.robot.shuffleboard.DrivetrainDiagnosticsShuffleboard;
import frc.util.Deadband;
import frc.util.SwerveModuleConverter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.geom.Point2D;

import static frc.robot.RobotMap.*;

import java.util.List;

// Template From: https://github.com/SwerveDriveSpecialties/swerve-template/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java
public class DrivetrainSubsystem extends DrivetrainSubsystemBase {
  
  // Since Mk4ModuleBuilder has been deprecated, this code was taken from that folder to account for the missing Gear Ratios
  public enum GearRatio {
    L1(SdsModuleConfigurations.MK4_L1),
    L2(SdsModuleConfigurations.MK4_L2),
    L3(SdsModuleConfigurations.MK4_L3),
    L4(SdsModuleConfigurations.MK4_L4);

    private final MechanicalConfiguration configuration;

    GearRatio(MechanicalConfiguration configuration) {
        this.configuration = configuration;
    }

    public MechanicalConfiguration getConfiguration() {
        return configuration;
    }
  }

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private Pose2d _targetPose = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23), Rotation2d.fromDegrees(-180));

  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;
  // private final SwerveDriveOdometry _odometryFromKinematics;
  private final SwerveDriveOdometry  _odometryFromHardware;
  // private final DrivetrainDiagnosticsShuffleboard _diagnostics;

  // private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] _moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

  // private final DriveCharacteristics _driveCharacteristics;

  private Pose2d _markedPosition = null;
  private Field2d _field2d = new Field2d();

  public DrivetrainSubsystem() {
    MkModuleConfiguration moduleConfig = new MkModuleConfiguration();
      moduleConfig.setSteerCurrentLimit(30.0);
      moduleConfig.setDriveCurrentLimit(40.0);
      moduleConfig.setSteerPID(0.2, 0.0, 0.1);

    // SmartDashboard.putNumber("GearRatio L1 wheel diameter", GearRatio.L1.getConfiguration().getWheelDiameter());
    // SmartDashboard.putNumber("GearRatio L1 drive reduction", GearRatio.L1.getConfiguration().getDriveReduction());

    m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(GearRatio.L1.getConfiguration())
      .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(GearRatio.L1.getConfiguration())
      .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
      .build();

    m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(GearRatio.L1.getConfiguration())
      .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(GearRatio.L1.getConfiguration())
      .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
      .build();

    double swerveDriveDelay = 0;
    double swerveRotateDelay = 0.25;

    ((CANSparkMax) m_frontLeftModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay); 
    ((CANSparkMax) m_frontRightModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);
    ((CANSparkMax) m_backLeftModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);
    ((CANSparkMax) m_backRightModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);

    ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay); 
    ((CANSparkMax) m_frontRightModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);
    ((CANSparkMax) m_backLeftModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);
    ((CANSparkMax) m_backRightModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);

    /*_odometryFromKinematics = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), 
    new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_backLeftModule.getPosition(),
      m_backRightModule.getPosition()
    }, new Pose2d(0, 0, new Rotation2d()));*/

    _odometryFromHardware = new SwerveDriveOdometry(
      m_kinematics, this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, new Pose2d(0, 0, new Rotation2d()));
    // _diagnostics = new DrivetrainDiagnosticsShuffleboard();
    // _driveCharacteristics = new DriveCharacteristics();

    SmartDashboard.putData("HardwareOdometry Field", _field2d);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  @Override
  public void zeroGyroscope() {
    m_navx.zeroYaw();
    // _odometryFromKinematics.resetPosition(new Pose2d(0, 0, new Rotation2d()), this.getGyroscopeRotation());
    var hardwarePose = _odometryFromHardware.getPoseMeters();
    _odometryFromHardware.resetPosition(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, new Pose2d(hardwarePose.getTranslation(), new Rotation2d()));
    // _driveCharacteristics.reset();

    Robot.POSE_ESTIMATOR_SUBSYSTEM.zeroGyroscope();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeftModule.getPosition();
    positions[1] = m_frontRightModule.getPosition();
    positions[2] = m_backLeftModule.getPosition();
    positions[3] = m_backRightModule.getPosition();
    return positions;
 }

  @Override
  public Rotation2d getOdometryRotation() {
    return _odometryFromHardware.getPoseMeters().getRotation();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getAngle());
  }

  @Override
  public Rotation2d getGyroscopeRotation2dTest() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getAngle());
  }

  @Override
  public void holdPosition() {
    // create an X pattern with the wheels to thwart pushing from other robots
    _moduleStates[0].angle = Rotation2d.fromDegrees(45); // front left
    _moduleStates[0].speedMetersPerSecond = 0;
    _moduleStates[1].angle = Rotation2d.fromDegrees(-45); // front right
    _moduleStates[1].speedMetersPerSecond = 0;
    _moduleStates[2].angle = Rotation2d.fromDegrees(-45); // back left
    _moduleStates[2].speedMetersPerSecond = 0;
    _moduleStates[3].angle = Rotation2d.fromDegrees(45); // back right
    _moduleStates[3].speedMetersPerSecond = 0;
  }

  @Override
  public void drive(ChassisSpeeds targetChassisSpeeds) {
    _moduleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = _moduleStates; // states and _modulestates still point to the same data
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    /*_odometryFromKinematics.update(this.getGyroscopeRotation(), new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_backLeftModule.getPosition(),
      m_backRightModule.getPosition()
    });*/

    // _diagnostics.updateKinematics(_odometryFromKinematics, states);

    var statesHardware = new SwerveModuleState[4];
    statesHardware[0] = SwerveModuleConverter.ToSwerveModuleState(m_frontLeftModule, 0);
    statesHardware[1] = SwerveModuleConverter.ToSwerveModuleState(m_frontRightModule, 0);
    statesHardware[2] = SwerveModuleConverter.ToSwerveModuleState(m_backLeftModule, 0);
    statesHardware[3] = SwerveModuleConverter.ToSwerveModuleState(m_backRightModule, 0);

    // var odometryStates = DrivetrainSubsystem.adjustStatesForOdometry(statesHardware);
    _odometryFromHardware.update(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      });

    // _diagnostics.updateHardware(_odometryFromHardware, statesHardware);

    Deadband.adjustRotationWhenStopped(states, statesHardware);
    
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    // _driveCharacteristics.update(_odometryFromHardware.getPoseMeters(), 360 - m_navx.getAngle());

    setRobotZoneFromOdometry();
    _field2d.setRobotPose(_odometryFromHardware.getPoseMeters());

    var roll = this.getRoll();
    SmartDashboard.putNumber("roll", roll);
  }

  public boolean drivetrainIsAtTargetCoordinates() {
    boolean isAtTarget = false;

    double xDifference = Math.abs(getTargetPoseX() - getPoseX());
    double yDifference = Math.abs(getTargetPoseY() - getPoseY());

    if (xDifference <= Constants.COORDINATE_MATCHES_MARGIN_METERS && yDifference <= Constants.COORDINATE_MATCHES_MARGIN_METERS) {
      isAtTarget = true;
    }

    return isAtTarget;
  }

  public boolean drivetrainIsAtTargetPose() {
    boolean isAtCoordinates = drivetrainIsAtTargetCoordinates();

    Rotation2d rotationDifference = getPoseRotation().minus(getTargetPoseRotation());
    double rotationDifferenceDegrees = Math.abs(rotationDifference.getDegrees());
    boolean isAtRotation = rotationDifferenceDegrees <= Constants.ROTATION_MATCHES_MARGIN_DEGREES;

    return isAtCoordinates && isAtRotation;
  }

  public Pose2d getTargetPose() {
    return _targetPose;
  }

  public double getTargetPoseX() {
    return _targetPose.getX();
  }

  public double getTargetPoseY() {
    return _targetPose.getY();
  }

  public Rotation2d getTargetPoseRotation() {
    return _targetPose.getRotation();
  }

  public void setTargetPose(Pose2d targetPose) {
    this._targetPose = targetPose;
  }

  /**
   * Adjusts the velocit of the states to account for reduction in wheel diameter due to wear.
   * @param states the original hardware states
   * @return hardware states adjusted for reduced wheel diameter
   */
  private static SwerveModuleState[] adjustStatesForOdometry(SwerveModuleState[] states) {
    var adjustedStates = new SwerveModuleState[4];
    double multiplier = Constants.SWERVE_ODOMETRY_MULTIPLIER;
    adjustedStates[0] = new SwerveModuleState(states[0].speedMetersPerSecond * multiplier, states[0].angle);
    adjustedStates[1] = new SwerveModuleState(states[1].speedMetersPerSecond * multiplier, states[1].angle);
    adjustedStates[2] = new SwerveModuleState(states[2].speedMetersPerSecond * multiplier, states[2].angle);
    adjustedStates[3] = new SwerveModuleState(states[3].speedMetersPerSecond * multiplier, states[3].angle);
    return states;
  }

  public Pose2d getPose() {
    return _odometryFromHardware.getPoseMeters();
  }

  public double getPoseX() {
    return getPose().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }
  
  public Rotation2d getPoseRotation() {
    return getPose().getRotation();
  }

  private void setRobotZoneFromOdometry() {
    double robotX = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX();
    double robotY = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY();

    // If the odometry override is active, set the coords to -1, -1.
    if (Robot.ODOMETRY_OVERRIDE) {
      robotX = -1;
      robotY = -1;
    }

    Point2D robotPoint = new Point2D.Double(robotX, robotY);

    Robot.fieldSubzone = Robot.fieldZones.getPointFieldZone(robotPoint);
  }

  public void resetOdometryCurrentPosition() {
    resetOdometry(getPose());
  }

  public void resetOdometry(Pose2d pose) {
    Transform2d transform = new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI));
    //pose.transformBy(transform);
    var targetPose = new Pose2d(pose.getTranslation(), pose.getRotation());


    // _odometryFromKinematics.resetPosition(targetPose, this.getGyroscopeRotation());
    _odometryFromHardware.resetPosition(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, targetPose);

    Robot.POSE_ESTIMATOR_SUBSYSTEM.resetPosition(targetPose);
  }

  // used only by SwerveControllerCommand to follow trajectories
  private void setModuleStates(SwerveModuleState[] moduleStates) {
    _moduleStates = moduleStates;
  }

  private void stop() {
    this.drive(new ChassisSpeeds(0,0,0));
  }

  public boolean isRobotSquareWithField() {
    // TODO: Implement this method
    return false;
  }

  @Override
  public double getRoll() {
    return m_navx.getRoll();
  }

  public double getPitch() {
    return m_navx.getPitch();
  }

  @Override
  public TrajectoryConfig GetTrajectoryConfig() {
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND,
      Constants.TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_kinematics);

    return config;
  }

  @Override
  public Command CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory trajectory) {
    return new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
  }

  @Override
  public Command CreateFollowTrajectoryCommand(Trajectory trajectory) {
    return CreateFollowTrajectoryCommand(trajectory, false);
  }

  @Override
  public Command CreateFollowTrajectoryCommandSwerveOptimized(Trajectory trajectory) {
    return CreateFollowTrajectoryCommand(trajectory, true);
  }
  
  private Command CreateFollowTrajectoryCommand(Trajectory trajectory, boolean swerveOptimized) {
    var robotAngleController =
        new ProfiledPIDController(
          Constants.SWERVE_CONTROLLER_ANGLE_KP, 0, 0, Constants.SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS);
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

    Command followTrajectory;
    if (swerveOptimized) {
      followTrajectory = CreateSwerveCommandWhichImmediatelyRotatesToRotationOfLastPointInTrajectory(trajectory, robotAngleController);
    } else {
      followTrajectory = CreateSwerveCommandWhichRespectsTheRotationOfEachPoint(trajectory, robotAngleController);
    }

    return followTrajectory
      .andThen(this::stop);
  }

  private RavenSwerveControllerCommand CreateSwerveCommandWhichRespectsTheRotationOfEachPoint(Trajectory trajectory, ProfiledPIDController robotAngleController) {
    return new RavenSwerveControllerCommand(
      trajectory,
      this::getPose, // Functional interface to feed supplier
      m_kinematics,

      // Position controllers
      new PIDController(Constants.SWERVE_CONTROLLER_X_KP, 0, 0),
      new PIDController(Constants.SWERVE_CONTROLLER_Y_KP, 0, 0),
      robotAngleController,
      this::setModuleStates,
      this);
  }

  private SwerveControllerCommand CreateSwerveCommandWhichImmediatelyRotatesToRotationOfLastPointInTrajectory(Trajectory trajectory, ProfiledPIDController robotAngleController) {
    return new SwerveControllerCommand(
      trajectory,
      this::getPose, // Functional interface to feed supplier
      m_kinematics,

      // Position controllers
      new PIDController(Constants.SWERVE_CONTROLLER_X_KP, 0, 0),
      new PIDController(Constants.SWERVE_CONTROLLER_Y_KP, 0, 0),
      robotAngleController,
      this::setModuleStates,
      this);
  }

  @Override
  public Command getMarkPositionCommand() {
    return new InstantCommand(() -> {
      var pos = this.getPose();
      _markedPosition = new Pose2d(pos.getTranslation(), pos.getRotation());
    });
  }

  @Override
  public Command getReturnToMarkedPositionCommand() {
    return new InstantCommand(() -> {
      var trajectoryConfig = this.GetTrajectoryConfig();
      var trajectory = TrajectoryGenerator.generateTrajectory(
        this.getPose(),
        List.of(),
        _markedPosition,
        trajectoryConfig);
  
      var cmd = this.CreateFollowTrajectoryCommandSwerveOptimized(trajectory);
      cmd.schedule();
    });
  }

  public boolean robotIsAtTargetCoordinates() {
    return robotIsAtTargetXCoordinate() && robotIsAtTargetYCoordinate();
  }

  public boolean robotIsAtTargetXCoordinate() {
    boolean isWithinErrorMargin = false;

    var target = Robot.TABLET_SCORING_SUBSYSTEM.GetScoringCoordinates();
    if (target == null) {
        return false;
    }

    double robotX = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX();

    if (Math.abs(robotX - target.getX()) < Constants.ROBOT_IS_ALIGNED_ERROR_MARGIN_METERS) {
      isWithinErrorMargin = true;
    }

    return isWithinErrorMargin;
  }

  public boolean robotIsAtTargetYCoordinate() {
    boolean isWithinErrorMargin = false;

    var target = Robot.TABLET_SCORING_SUBSYSTEM.GetScoringCoordinates();
    if (target == null) {
        return false;
    }

    double robotX = Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY();

    if (Math.abs(robotX - target.getY()) < Constants.ROBOT_IS_ALIGNED_ERROR_MARGIN_METERS) {
      isWithinErrorMargin = true;
    }

    return isWithinErrorMargin;
  }
}