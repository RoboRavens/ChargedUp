// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.util.arm.ArmSetpoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Rotation Motor Absolute Encoder Position
  public static final double ARM_ROTATION_ABSOLUTE_ENCODER_POSITION_AT_ZERO = 222;

  //LEDs
  public static final int TOTAL_LEDS_STRIP_LENGTH = 117;
  public static final int END_OF_FIRST_LEDS_SECTION = 28;
  public static final int END_OF_SECOND_LEDS_SECTION = 56;
  public static final int END_OF_THIRD_LEDS_SECTION = 84;
  public static final int END_OF_FOURTH_LEDS_SECTION = TOTAL_LEDS_STRIP_LENGTH;

  //Motion Magic
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 0;
  public static final Gains rotationGains = new Gains(1.35, 0.001, 0.00000, 0.0, 0, 1.0);
  public static final Gains extensionGains = new Gains(.8, 0.002, 0.00000, 0.0, 0, 1.0);
  
  // Position to degrees 4096 counts per revolution
  public static final int COUNTS_PER_REVOLUTION = 4096;
  public static final double ARM_DEGREES_TO_ENCODER_UNITS = COUNTS_PER_REVOLUTION / 360;
 

  // DRIVETRAIN PATHFINDING
  public static final double COORDINATE_MATCHES_MARGIN_METERS = Units.inchesToMeters(2);
  public static final double ROTATION_MATCHES_MARGIN_DEGREES = 3.0;
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
  
  // DRIVETRAIN SLEW
  public static final double SLEW_FRAMES_TO_MAX_X_VELOCITY = 10; // forward-back
  public static final double SLEW_FRAMES_TO_MAX_Y_VELOCITY = 10; // left-right
  public static final double SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY = 10; // turning

  // DRIVETRAIN ARM SCALING
  public static final double DRIVE_SPEED_SCALE_AT_ARM_MAX_EXTENSION = .2;

    
  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

  //CONTROLS
  public static final double AXIS_IS_PRESSED_VALUE = .25;
  // public static final double JOYSTICK_DEADBAND = .05;
  public static final double JOYSTICK_DEADBAND = .1;
  public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 3;

  // ARM
  // These numbers are based off of CAD measurements but have not been tested or measured IRL.
  public static final double FULCRUM_HEIGHT_INCHES = 19.25;
  public static final double FULCRUM_TO_FRAME_INCHES = 14;
  public static final double MAX_TELEOP_HEIGHT_INCHES = 78;
  public static final double MAX_TELEOP_HORIZONTAL_EXTENSION_INCHES = 48;
  public static final double ARM_BASE_LENGTH_INCHES = 33.524;
  public static final double ARM_EXTENDED_LENGTH_INCHES = 66.25;
  public static final double ARM_MAX_EXTENSION_ENCODER_UNITS = 125000; // This number is a wild guess and liable to be very wrong.
  public static final double SAFETY_MARGIN_INCHES = 1;
  public static final double ARM_EXTENSION_PER_UNIT = Constants.ARM_EXTENDED_LENGTH_INCHES / Constants.ARM_MAX_EXTENSION_ENCODER_UNITS;

  // The magnitude of power applied to the arm while under manual control.
  public static final double ARM_MANUAL_ROTATION_VOLTAGE = 1.5;
  public static final double ARM_MANUAL_EXTENSION_VOLTAGE = 1.5;

  // This is magnitude of the voltage output that will be applied to the arm rotation,
  // while it is being held by the brake, to fight backlash upon brake release.
  public static final double ARM_BRAKE_ANTI_BACKLASH_VOLTAGE = .5;

  // Used for torque calculations because the claw is light, so extending it adds minimal torque.
  public static final double ARM_BASE_LENGTH_CLAW_OPEN_INCHES = 26.353;
  public static final double ARM_EXTENDED_LENGTH_CLAW_OPEN_INCHES = 66.353;

  // When the claw is retracted, it's "0% extended" in terms of encoder ticks,
  // but it still has some non-zero percent of its max length, because it physically exists.
  // This constant is that percent of the max length.
  public static final double ARM_MINIMUM_EXTENSION_RATIO = ARM_BASE_LENGTH_CLAW_OPEN_INCHES / ARM_EXTENDED_LENGTH_CLAW_OPEN_INCHES;

  // Calculate the limits for horizontal and vertical expansion based on the fulcrum's location.
  public static final double VERTICAL_EXPANSION_ROOM = Constants.MAX_TELEOP_HEIGHT_INCHES - Constants.SAFETY_MARGIN_INCHES;
  public static final double HORIZONTAL_EXPANSION_ROOM = Constants.FULCRUM_TO_FRAME_INCHES + Constants.MAX_TELEOP_HORIZONTAL_EXTENSION_INCHES - Constants.SAFETY_MARGIN_INCHES;

  public static final double ARM_MAX_ROTATION_DEGREES = 120;
  public static final double ARM_STARTING_DEGREES = 0;

  public static final int ARM_FULL_RETRACT_ROTATION_ANGLE = 0;
  public static final int ARM_GROUND_PICKUP_ROTATION_ANGLE = 119;

  public static final int ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_ANGLE = 64;

  public static final double ARM_DOUBLE_SUBSTATION_CONE_PICKUP_ROTATION_ANGLE = 63;
  public static final double ARM_DOUBLE_SUBSTATION_CUBE_PICKUP_ROTATION_ANGLE = ARM_DOUBLE_SUBSTATION_CONE_PICKUP_ROTATION_ANGLE;
  public static final int ARM_SCORE_LOW_ROTATION_ANGLE = 45;
  public static final int ARM_SCORE_CONE_MID_ROTATION_ANGLE = 54;
  public static final int ARM_SCORE_CUBE_MID_ROTATION_ANGLE = 65;
  public static final int ARM_SCORE_CONE_HIGH_ROTATION_ANGLE = 57;
  public static final int ARM_SCORE_CUBE_HIGH_ROTATION_ANGLE = 60;

  public static final double ARM_SEQUENCED_RETRACTION_STAGING_ANGLE = 45;
  public static final double ARM_SEQUENCED_RETRACTION_STAGING_EXTENSION_SETPOINT = 20000;
  public static final ArmSetpoint ARM_SEQUENCED_RETRACTION_STAGING_SETPOINT = new ArmSetpoint("Staging Retract", ARM_SEQUENCED_RETRACTION_STAGING_EXTENSION_SETPOINT, ARM_SEQUENCED_RETRACTION_STAGING_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final ArmSetpoint ARM_SEQUENCED_RETRACTION_FROM_REVERSE_STAGING_SETPOINT = new ArmSetpoint("Staging Retract - Reverse", ARM_SEQUENCED_RETRACTION_STAGING_EXTENSION_SETPOINT, ARM_SEQUENCED_RETRACTION_STAGING_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS * -1);

  public static final int ARM_FULL_RETRACT_EXTENSION_SETPOINT = 0;
  public static final int ARM_FULL_RETRACT_ROTATION_SETPOINT = (int) Math.round(ARM_FULL_RETRACT_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_GROUND_PICKUP_EXTENSION_SETPOINT = 0;
  public static final int ARM_GROUND_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_GROUND_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_ROTATION_MAXIMUM_ENCODER_UNITS = ARM_GROUND_PICKUP_ROTATION_SETPOINT;
  public static final int ARM_SINGLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT = 0;
  public static final int ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_DOUBLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT = 70000;
  public static final int ARM_DOUBLE_SUBSTATION_CONE_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_DOUBLE_SUBSTATION_CONE_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_DOUBLE_SUBSTATION_CUBE_PICKUP_ROTATION_SETPOINT = (int) Math.round(ARM_DOUBLE_SUBSTATION_CUBE_PICKUP_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_LOW_EXTENSION_SETPOINT = 0;
  public static final int ARM_SCORE_LOW_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_LOW_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CONE_MID_EXTENSION_SETPOINT = 60000;
  public static final int ARM_SCORE_CONE_MID_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CONE_MID_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CUBE_MID_EXTENSION_SETPOINT = 36500;
  public static final int ARM_SCORE_CUBE_MID_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CUBE_MID_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT = 119000;
  public static final int ARM_SCORE_CONE_HIGH_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CONE_HIGH_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);
  public static final int ARM_SCORE_CUBE_HIGH_EXTENSION_SETPOINT = 108000;
  public static final int ARM_SCORE_CUBE_HIGH_ROTATION_SETPOINT = (int) Math.round(ARM_SCORE_CUBE_HIGH_ROTATION_ANGLE * ARM_DEGREES_TO_ENCODER_UNITS);

  public static final int ARM_SCORE_CONE_HIGH_OPPOSITE_ROTATION_SETPOINT = (int) Math.round((ARM_SCORE_CONE_HIGH_ROTATION_ANGLE * -1) * ARM_DEGREES_TO_ENCODER_UNITS);

  public static final ArmSetpoint ARM_EXTENSION_TEST_SETPOINT = new ArmSetpoint("Arm Extend", ARM_MAX_EXTENSION_ENCODER_UNITS, ARM_FULL_RETRACT_ROTATION_SETPOINT);


  public static final ArmSetpoint ARM_SEQUENCED_EXTENSION_STAGING_SETPOINT = new ArmSetpoint("Staging Extend", 60000, 45 * ARM_DEGREES_TO_ENCODER_UNITS * -1);
  

  
  public static final ArmSetpoint ARM_FULL_RETRACT_SETPOINT = new ArmSetpoint("Full Retract", ARM_FULL_RETRACT_EXTENSION_SETPOINT, ARM_FULL_RETRACT_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_GROUND_PICKUP_SETPOINT = new ArmSetpoint("Ground Pickup", ARM_GROUND_PICKUP_EXTENSION_SETPOINT, ARM_GROUND_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SINGLE_SUBSTATION_PICKUP_SETPOINT = new ArmSetpoint("Single Substation Pickup", ARM_SINGLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT, ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_DOUBLE_SUBSTATION_CONE_PICKUP_SETPOINT = new ArmSetpoint("Double Substation Pickup", ARM_DOUBLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT, ARM_DOUBLE_SUBSTATION_CONE_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_DOUBLE_SUBSTATION_CUBE_PICKUP_SETPOINT = new ArmSetpoint("Double Substation Pickup", ARM_DOUBLE_SUBSTATION_PICKUP_EXTENSION_SETPOINT, ARM_DOUBLE_SUBSTATION_CUBE_PICKUP_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_LOW_SETPOINT = new ArmSetpoint("Score Low", ARM_SCORE_LOW_EXTENSION_SETPOINT, ARM_SCORE_LOW_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CONE_MID_SETPOINT = new ArmSetpoint("Score Cone Mid", ARM_SCORE_CONE_MID_EXTENSION_SETPOINT, ARM_SCORE_CONE_MID_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CUBE_MID_SETPOINT = new ArmSetpoint("Score Cube Mid", ARM_SCORE_CUBE_MID_EXTENSION_SETPOINT, ARM_SCORE_CUBE_MID_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CONE_HIGH_SETPOINT = new ArmSetpoint("Score Cone High", ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT, ARM_SCORE_CONE_HIGH_ROTATION_SETPOINT);
  public static final ArmSetpoint ARM_SCORE_CUBE_HIGH_SETPOINT = new ArmSetpoint("Score Cube High", ARM_SCORE_CUBE_HIGH_EXTENSION_SETPOINT, ARM_SCORE_CUBE_HIGH_ROTATION_SETPOINT);


  public static final ArmSetpoint ARM_REVERSE_TEST_SETPOINT = new ArmSetpoint("Reverse Test", ARM_FULL_RETRACT_EXTENSION_SETPOINT, ARM_SINGLE_SUBSTATION_PICKUP_ROTATION_SETPOINT * -1);


  public static final ArmSetpoint ARM_SCORE_CONE_MID_REVERSE_SETPOINT = new ArmSetpoint("Score Cone Mid - Reverse", ARM_SCORE_CONE_MID_EXTENSION_SETPOINT, ARM_SCORE_CONE_MID_ROTATION_SETPOINT * -1);
  public static final ArmSetpoint ARM_SCORE_CONE_HIGH_REVERSE_SETPOINT = new ArmSetpoint("Score Cone High", ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT, ARM_SCORE_CONE_HIGH_OPPOSITE_ROTATION_SETPOINT);
  
  public static final ArmSetpoint ARM_SCORE_CONE_HIGH_REVERSE_STAGING_SETPOINT = new ArmSetpoint("Score Cone High Staging", ARM_SCORE_CONE_HIGH_EXTENSION_SETPOINT, 0);
 
  

  public static final double ARM_ROTATION_MANUAL_DEGREES_PER_SECOND = 30;
  public static final double ARM_ROTATION_MANUAL_NATIVE_UNITS_PER_TICK = ARM_ROTATION_MANUAL_DEGREES_PER_SECOND * ARM_DEGREES_TO_ENCODER_UNITS / 50;

  public static final double ARM_EXTENSION_MANUAL_NATIVE_UNITS_PER_SECOND = 50000;
  public static final double ARM_EXTENSION_MANUAL_NATIVE_UNITS_PER_TICK = ARM_EXTENSION_MANUAL_NATIVE_UNITS_PER_SECOND / 50;



  public static final double ARM_ROTATION_VELOCITY = 198;
  public static final double ARM_ROTATION_ACCELERATION = 4000;
  public static final double ARM_EXTENSION_VELOCITY = 10000;
  public static final double ARM_EXTENSION_ACCELERATION = 10000;

  public static final double ARM_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = 500;
  public static final double ARM_EXTENSION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS = 500;
  public static final double ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 200;
  public static final double ARM_EXTENSION_TIMEOUT_ENCODER_TICKS_PER_SECOND = 10000;
  public static final double ARM_TIMEOUT_BASE_VALUE = 1;
  // public static final double ARM_EXTENSION_TIMEOUT_BASE_VALUE = .25;
  
  public static final double EXTENSION_UPRIGHT_EMPTY_AFF = .1;
  public static final double EXTENSION_UPRIGHT_LOADED_AFF = .11;
  public static final double ROTATION_SIDEWAYS_EMPTY_AFF = .05;
  public static final double ROTATION_SIDEWAYS_LOADED_AFF = .055;

  // CLAW
  public static final double CLAW_CLOSE_TIMEOUT_SECONDS = .35;
  public static final double CLAW_OPEN_TIMEOUT_SECONDS = .75;
  public static final double AUTO_CLAW_CLOSE_TIMEOUT_SECONDS = CLAW_CLOSE_TIMEOUT_SECONDS;
  public static final double AUTO_CLAW_OPEN_TIMEOUT_SECONDS = CLAW_OPEN_TIMEOUT_SECONDS;
  public static final double RUMBLE_TIME = 0.2;
  public static final double CLAW_LOCKOUT_DURATION_SECONDS = 1;

  // Limelight/vision-based odometry
  public static final double STATE_STANDARD_DEVIATION = .1;
  public static final double STARTING_VISION_STANDARD_DEVIATION = .9;
  public static final double MINIMUM_VISION_STANDARD_DEVIATION = .05;
  public static final double ROBOT_IS_ALIGNED_ERROR_MARGIN_METERS = Units.inchesToMeters(2.5);

  // Auto balancing
  public static final double AUTO_BALANCE_APPROACH_DRIVE_SPEED_METERS_PER_SECOND = 2;
  public static final double AUTO_BALANCE_BALANCE_DRIVE_SPEED_METERS_PER_SECOND = 2;
  public static final double ROBOT_IS_TILTED_DEGREES = 10;
  public static final double ROBOT_IS_LEVEL_DEGREES = 4;
}
