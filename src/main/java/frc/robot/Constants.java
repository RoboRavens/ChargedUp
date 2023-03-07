// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.ArmSetpoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Motion Magic
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 0;
  public static final Gains kGains = new Gains(0.09, 0.000009, 0.000005, 0.0, 0, 1.0);
  // Position to degrees 4096 counts per revolution
  public static final int COUNTS_PER_REVOLUTION = 4096;
  public static final double ARM_DEGREES_TO_ENCODER_UNITS = COUNTS_PER_REVOLUTION / 360;
 

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

    
  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

  //CONTROLS
  public static final double AXIS_IS_PRESSED_VALUE = .25;
  // public static final double JOYSTICK_DEADBAND = .05;
  public static final double JOYSTICK_DEADBAND = .05;
  public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 3;

  // ARM
  // These numbers are based off of CAD measurements but have not been tested or measured IRL.
  public static final double FULCRUM_HEIGHT_INCHES = 19.25;
  public static final double FULCRUM_TO_FRAME_INCHES = 14;
  public static final double MAX_TELEOP_HEIGHT_INCHES = 78;
  public static final double MAX_TELEOP_HORIZONTAL_EXTENSION_INCHES = 48;
  public static final double ARM_BASE_LENGTH_INCHES = 33.524;
  public static final double ARM_EXTENDED_LENGTH_INCHES = 66.25;
  public static final double ARM_MAX_EXTENSION_ENCODER_UNITS = 10000; // This number is a wild guess and liable to be very wrong.
  public static final double SAFETY_MARGIN_INCHES = 1;
  public static final double ARM_EXTENSION_PER_UNIT = Constants.ARM_EXTENDED_LENGTH_INCHES / Constants.ARM_MAX_EXTENSION_ENCODER_UNITS;

  // Calculate the limits for horizontal and vertical expansion based on the fulcrum's location.
  public static final double VERTICAL_EXPANSION_ROOM = Constants.MAX_TELEOP_HEIGHT_INCHES - Constants.SAFETY_MARGIN_INCHES;
  public static final double HORIZONTAL_EXPANSION_ROOM = Constants.FULCRUM_TO_FRAME_INCHES + Constants.MAX_TELEOP_HORIZONTAL_EXTENSION_INCHES - Constants.SAFETY_MARGIN_INCHES;

  public static final double ARM_MAX_ROTATION_DEGREES = 120;
  public static final double ARM_STARTING_DEGREES = 0;

  public static final int ARM_FULL_RETRACT_ROTATION_ANGLE = 0;
  public static final int ARM_GROUND_PICKUP_ROTATION_ANGLE = 114;
  public static final int ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_ANGLE = 75;
  public static final int ARM_DOUBLE_SUBSTATION_PICKUP_ROTATION_ANGLE = 60;
  public static final int ARM_SCORE_LOW_ROTATION_ANGLE = 114;
  public static final int ARM_SCORE_CONE_MID_ROTATION_ANGLE = 55;
  public static final int ARM_SCORE_CUBE_MID_ROTATION_ANGLE = 70;
  public static final int ARM_SCORE_CONE_HIGH_ROTATION_ANGLE = 55;
  public static final int ARM_SCORE_CUBE_HIGH_ROTATION_ANGLE = 60;

  public static final int ARM_FULL_RETRACT_EXTENSION_SETPOINT = 0;
  public static final int ARM_FULL_RETRACT_ROTATION_SETPOINT = (int) Math.round(ARM_FULL_RETRACT_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_GROUND_PICKUP_EXTENSION_SETPOINT = 0;
  public static final int ARM_GROUND_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_GROUND_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SINGLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT = 0;
  public static final int ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_DOUBLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT = 0;
  public static final int ARM_DOUBLE_SUBSTATION_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_DOUBLE_SUBSTATION_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_LOW_EXTENSION_SETPOINT = 0;
  public static final int ARM_SCORE_LOW_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_LOW_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CONE_MID_EXTENSION_SETPOINT = 100;
  public static final int ARM_SCORE_CONE_MID_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CONE_MID_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CUBE_MID_EXTENSION_SETPOINT = 100;
  public static final int ARM_SCORE_CUBE_MID_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CUBE_MID_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT = 200;
  public static final int ARM_SCORE_CONE_HIGH_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CONE_HIGH_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CUBE_HIGH_EXTENSION_SETPOINT = 200;
  public static final int ARM_SCORE_CUBE_HIGH_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CUBE_HIGH_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);

  public static final ArmSetpoint ARM_FULL_RETRACT_SETPOINT = new ArmSetpoint("Full Retract", ARM_FULL_RETRACT_EXTENSION_SETPOINT, ARM_FULL_RETRACT_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_GROUND_PICKUP_SETPOINT = new ArmSetpoint("Ground Pickup", ARM_GROUND_PICKUP_EXTENSION_SETPOINT, ARM_GROUND_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SINGLE_SUBSTATION_PICKUP_SETPOINT = new ArmSetpoint("Single Substation Pickup", ARM_SINGLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT, ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_DOUBLE_SUBSTATION_PICKUP_SETPOINT = new ArmSetpoint("Double Substation Pickup", ARM_DOUBLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT, ARM_DOUBLE_SUBSTATION_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_LOW_SETPOINT = new ArmSetpoint("Score Low", ARM_SCORE_LOW_EXTENSION_SETPOINT, ARM_SCORE_LOW_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CONE_MID_SETPOINT = new ArmSetpoint("Score Cone Mid", ARM_SCORE_CONE_MID_EXTENSION_SETPOINT, ARM_SCORE_CONE_MID_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CUBE_MID_SETPOINT = new ArmSetpoint("Score Cube Mid", ARM_SCORE_CUBE_MID_EXTENSION_SETPOINT, ARM_SCORE_CUBE_MID_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CONE_HIGH_SETPOINT = new ArmSetpoint("Score Cone High", ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT, ARM_SCORE_CONE_HIGH_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CUBE_HIGH_SETPOINT = new ArmSetpoint("Score Cube High", ARM_SCORE_CUBE_HIGH_EXTENSION_SETPOINT, ARM_SCORE_CUBE_HIGH_ROTATION_SETPOINT);

  public static final double ARM_ROTATION_VELOCITY = 3000;
  public static final double ARM_ROTATION_ACCELERATION = 800;
  public static final double ARM_EXTENSION_VELOCITY = 1000;
  public static final double ARM_EXTENION_ACCELERATION = 100;

  public static final double ARM_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = 50;
  public static final double ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 2000;
  public static final double ARM_EXTENSION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 1000;
  public static final double ARM_EXTENSION_TIMEOUT_BASE_VALUE = .25;
  

  // CLAW
  public static final double CLAW_CLOSE_TIMEOUT_SECONDS = .25;
  public static final double CLAW_OPEN_TIMEOUT_SECONDS = .25;
}
