// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kTrackwidthMeters = 0.0381; // The tread width is 1.5 inches
  public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
        
  // DRIVETRAIN PATHFINDING
  public static final double TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND = 1.5;
  public static final double TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND = .6;
  public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
  public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI * .75;

  public static final double SWERVE_CONTROLLER_X_KP = 2;
  public static final double SWERVE_CONTROLLER_Y_KP = 2;
  public static final double SWERVE_CONTROLLER_ANGLE_KP = 4;

  private static final double SWERVE_WHEEL_DIAMETER = SdsModuleConfigurations.MK4_L1.getWheelDiameter();
  private static final double SWERVE_WHEEL_DIAMETER_INCREMENT_5BALL = 0.0003186; // reducing the wheel diameter by this increment adds one inch to the 5 ball auto
  private static final double SWERVE_5BALL_INCHES_OFFSET = 0; // positive number gets robot closer to human player station
  public static final double SWERVE_ODOMETRY_MULTIPLIER =
    (SWERVE_WHEEL_DIAMETER - (SWERVE_WHEEL_DIAMETER_INCREMENT_5BALL * SWERVE_5BALL_INCHES_OFFSET)) / SWERVE_WHEEL_DIAMETER;

  // Use this value when creating a PathWeaver project for our current robot
  public static final double TRACK_WIDTH_METERS = 0.6096;
    
  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

  //CONTROLS
  public static final double AXIS_IS_PRESSED_VALUE = .25;
  // public static final double JOYSTICK_DEADBAND = .05;
  public static final double JOYSTICK_DEADBAND = .05;
  public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 3;
}
