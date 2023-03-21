// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.controls.GamepadPOV;
import frc.robot.commands.LEDs.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.commands.claw.RumbleCommand;
import frc.robot.commands.drivetrain.DriveTwoInchesCommand;
import frc.robot.commands.drivetrain.DrivetrainChargeStationBalancingCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.util.Colors;
import frc.util.StateManagement;
import frc.util.StateManagement.ArmExtensionState;
import frc.util.StateManagement.ArmRotationState;
import frc.util.StateManagement.ClawState;
import frc.util.StateManagement.ZoneState;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.LimelightState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.PieceState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.ZoneState;
import frc.robot.subsystems.DrivetrainSubsystemMock;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightTrajectorySubsystem;
import frc.robot.subsystems.TrajectoryTestingSubsystem;
import frc.robot.subsystems.TabletScoring.ScoringShape;
import frc.robot.subsystems.TabletScoring.TabletScoringResult;
import frc.robot.subsystems.TabletScoring.TabletScoringSubsystem;
import frc.util.field.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  public static final DrivetrainSubsystem DRIVE_TRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  //public static final DrivetrainSubsystemBase DRIVETRAIN_SUBSYSTEM_BASE = new DrivetrainSubsystemMock(); 
  public static final TrajectoryTestingSubsystem TRAJECTORY_TESTING_SUBSYSTEM = new TrajectoryTestingSubsystem();
  public static final DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  public static final Joystick JOYSTICK = new Joystick(0);
  public static final Gamepad GAMEPAD = new Gamepad(JOYSTICK);
  public static final Gamepad OP_PAD_BUTTONS = new Gamepad(3);
  public static final Gamepad OP_PAD_SWITCHES = new Gamepad(2);
  public static final LimelightTrajectorySubsystem LIMELIGHT_TRAJECTORY_SUBSYSTEM = new LimelightTrajectorySubsystem();
  public static final DrivetrainChargeStationBalancingCommand chargeStationBalancingCommand = new DrivetrainChargeStationBalancingCommand();
  // public static GamePieceState gamePieceState = GamePieceState.CLEAR;
  // public static RowSelectionState rowSelectionState = RowSelectionState.CLEAR;
  // public static PieceRetrievalState pieceRetrievalState = PieceRetrievalState.CLEAR;
  public static final ArmSubsystem ARM_SUBSYSTEM = new ArmSubsystem();
  public static final ClawSubsystem CLAW_SUBSYSTEM = new ClawSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final TabletScoringSubsystem TABLET_SCORING_SUBSYSTEM = new TabletScoringSubsystem();
  public static final RumbleCommand RUMBLE_COMMAND = new RumbleCommand();
  // public static final StateManagement STATE_MANAGEMENT = new StateManagement();
  public static final LEDsSubsystem LED_SUBSYSTEM = new LEDsSubsystem();
  public static boolean driverControlOverride = false;
  public static boolean limelightOverride = false;

  public LEDsRainbowCommand ledsRainbowCommand = new LEDsRainbowCommand(LED_SUBSYSTEM);
  public LEDsBlinkColorsCommand ledsBlinkColorsCommand = new LEDsBlinkColorsCommand(LED_SUBSYSTEM);
  public LEDsSolidColorCommand ledsSignalConeCommand = new LEDsSolidColorCommand(LED_SUBSYSTEM, Colors.ORANGE);
  public LEDsSolidColorCommand ledsSignalCubeCommand = new LEDsSolidColorCommand(LED_SUBSYSTEM, Colors.PURPLE);
  //public LEDsBlinkThenStopCommand ledsBlinkThenStopCommand = new LEDsBlinkThenStopCommand(LED_SUBSYSTEM, Colors.ORANGE)
  public LEDsSectionsCommand ledsSectionsCommand = new LEDsSectionsCommand(LED_SUBSYSTEM);

  // Sets the default robot mechanism states (may need to be changed)
  public static OverallState overallState = OverallState.EMPTY_TRANSIT;
  public static ArmRotationState armRotationState = ArmRotationState.COLLECT_GROUND;
  public static ArmExtensionState armExtensionState = ArmExtensionState.RETRACTED;
  public static PieceState pieceState = PieceState.NONE;
  public static LoadState loadState = LoadState.EMPTY;
  public static LoadTargetState loadTargetState = LoadTargetState.DOUBLE_SUBSTATION;
  public static DrivetrainState drivetrainState = DrivetrainState.FREEHAND;
  public static ClawState clawState = ClawState.CLOSED;
  public static LimelightState limelightState = LimelightState.TAG_TRACKING;
  public static ScoringTargetState scoringTargetState = ScoringTargetState.NONE;
  public static ZoneState zoneState = ZoneState.NEUTRAL;

  public static FieldZones fieldZones = new FieldZones();
  public static FieldSubzone fieldSubzone = FieldZones.noneSubzone;
  public static DriverStation.Alliance allianceColor = Alliance.Invalid;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(drivetrainDefaultCommand);
    configureButtonBindings();
    configureTriggers();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    LED_SUBSYSTEM.ledsBlinkThenStop(255,0,0, 0, 0, 0);
    //LED_SUBSYSTEM.ledsBlinkColorsFast(255, 0, 0, 0, 255, 0);
    //LED_SUBSYSTEM.ledsSolidColor(0, 255, 0);
    //LED_SUBSYSTEM.ledsBlinkColors(255, 0, 0, 0, 255, 0);
    //LED_SUBSYSTEM.rainbowLeds();
    //LED_SUBSYTEM.armAngleLeds();

    setDriverStationData();

    
    TabletScoringResult tabletResult = TABLET_SCORING_SUBSYSTEM.GetScoringTarget();
    SmartDashboard.putString("TABLET PIECE", tabletResult.GetShape().name());
    SmartDashboard.putNumber("TABLET ROW", tabletResult.GetPosition().GetRow());
    SmartDashboard.putNumber("TABLET COLUMN", tabletResult.GetPosition().GetColumn());
    
    // SmartDashboard.putNumber("Power", ARM_SUBSYSTEM.testPower);

    // ARM_SUBSYSTEM.setTestPower(SmartDashboard.getNumber("Power", 0));

    

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Odometry rotation (degrees)", DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getDegrees());
    SmartDashboard.putNumber("Gyroscope rotation (degrees)", DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation2dTest().getDegrees());
    setNonButtonDependentOverallStates();
    // Button input dependent states
    SmartDashboard.putString("Piece State", pieceState.toString());
    SmartDashboard.putString("Scoring Target State", scoringTargetState.toString());
    SmartDashboard.putString("Load Target State", loadTargetState.toString());
    SmartDashboard.putString("Zone State", zoneState.toString());
    SmartDashboard.putString("Load State", loadState.toString());
    // Other states
    SmartDashboard.putString("Overall State", overallState.toString());
    SmartDashboard.putString("Drivetrain State", drivetrainState.toString());
    SmartDashboard.putString("Scheduled Arm Command", ARM_SUBSYSTEM.getCurrentCommand() == null ? "No command" : ARM_SUBSYSTEM.getCurrentCommand().getName());
    SmartDashboard.putString("Scheduled Claw Command", CLAW_SUBSYSTEM.getCurrentCommand() == null ? "No command" : CLAW_SUBSYSTEM.getCurrentCommand().getName());

    SmartDashboard.putString("Scheduled Drivetrain Command", DRIVE_TRAIN_SUBSYSTEM.getCurrentCommand() == null ? "No command" : DRIVE_TRAIN_SUBSYSTEM.getCurrentCommand().getName());

    setZoneStateFromFieldZone();

    SmartDashboard.putString("FieldSubzone", fieldSubzone.getName());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //System.out.println("OUTPUT!");
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    setDriverStationData();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    setDriverStationData();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void setNonButtonDependentOverallStates() {
    // Check the load state of the robot.
    if (Robot.CLAW_SUBSYSTEM.detectsGamePiece() && Robot.clawState == ClawState.CLOSED) {
      Robot.loadState = LoadState.LOADED;
    }

    // If the left trigger is pressed,
    // Set the overall state to either scoring alignment or hps pickup based on the zone state
    if (GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
      drivetrainState = DrivetrainState.ROBOT_ALIGN;
    }
    // Set the drive state back to freehand when the left trigger is released
    else if (drivetrainState == DrivetrainState.ROBOT_ALIGN) {
      drivetrainState = DrivetrainState.FREEHAND;
    }
    // Changes the overall state to empty or loaded transit when the trigger is released
    else if (
        // When the trigger isn't pulled we should be in "normal" operation,
        // but on the off chance the trigger is released during a special state,
        // do not override that state. When the special states clear themselves,
        // this if will go back to successfully setting the new state.
        StateManagement.isManipulatingGamePiece(Robot.overallState, Robot.clawState) == false
        && (Robot.overallState == OverallState.FINAL_SCORING_ALIGNMENT || overallState == OverallState.DOUBLE_SUBSTATION_PICKUP)) {
      if (loadState == LoadState.LOADED) {
        overallState = OverallState.LOADED_TRANSIT;
      }
      else {
        overallState = OverallState.EMPTY_TRANSIT;
      }
    }
    // Sets the robot state to PREPARING_TO_SCORE only once when the robot has a piece and is in the alliance community.
    // This conditional is designed as such so it does not continuously set itself when a different overall state such as FINAL_SCORING_ALIGNMENT or SCORING is set.
    if (Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY && Robot.overallState == OverallState.LOADED_TRANSIT) {
      Robot.overallState = OverallState.PREPARING_TO_SCORE;
    }

    // If we were preparing to score but leave the community, undo the state.
    if (Robot.zoneState != ZoneState.ALLIANCE_COMMUNITY && Robot.overallState == OverallState.PREPARING_TO_SCORE) {
      Robot.overallState = OverallState.LOADED_TRANSIT;
    }

    // Same thing but for preparing to collect a game piece when entering the loading zone.
    if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.overallState == OverallState.EMPTY_TRANSIT) {
      Robot.overallState = OverallState.DOUBLE_SUBSTATION_PICKUP;
    }
  }

  private void configureButtonBindings() {
    GAMEPAD.getPOVTrigger(GamepadPOV.Right).toggleOnTrue(new DriveTwoInchesCommand('R'));
    GAMEPAD.getPOVTrigger(GamepadPOV.Up).toggleOnTrue(new DriveTwoInchesCommand('F'));
    GAMEPAD.getPOVTrigger(GamepadPOV.Down).toggleOnTrue(new DriveTwoInchesCommand('B'));
    GAMEPAD.getPOVTrigger(GamepadPOV.Left).toggleOnTrue(new DriveTwoInchesCommand('L'));

    GAMEPAD.getButton(ButtonCode.START).onTrue(new ClawOpenCommand());
    GAMEPAD.getButton(ButtonCode.BACK).onTrue(new ClawCloseCommand());
    
    // GAMEPAD.getButton(ButtonCode.A).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));
    // GAMEPAD.getButton(ButtonCode.B).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));
    // GAMEPAD.getButton(ButtonCode.X).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CUBE_MID_SETPOINT));
    // GAMEPAD.getButton(ButtonCode.B).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));
    // GAMEPAD.getButton(ButtonCode.LEFTSTICK).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT));
    // OP_PAD_SWITCHES.getButton(ButtonCode.RETRACT_ARM).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SINGLE_SUBSTATION_PICKUP_SETPOINT));

    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_GROUND_PICKUP_SETPOINT));
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).onFalse(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));
    OP_PAD_SWITCHES.getButton(ButtonCode.RETRACT_ARM).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SINGLE_SUBSTATION_PICKUP_SETPOINT));
    //OP_PAD_SWITCHES.getButton(ButtonCode.ROTATE_ARM_MAX_ROTATION).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_DOUBLE_SUBSTATION_PICKUP_SETPOINT));//
    //OP_PAD_SWITCHES.getButton(ButtonCode.ROTATE_ARM_TO_ZERO).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_LOW_SETPOINT));//

    // OP_PAD_SWITCHES.getButton(ButtonCode.SCORE_LOW).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_MID_SETPOINT));
    // OP_PAD_SWITCHES.getButton(ButtonCode.SCORE_MID).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CUBE_MID_SETPOINT));
    // OP_PAD_SWITCHES.getButton(ButtonCode.SCORE_HIGH).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT));

    OP_PAD_SWITCHES.getButton(ButtonCode.EJECT_PIECE).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CUBE_HIGH_SETPOINT));
    OP_PAD_BUTTONS.getButton(ButtonCode.SET_ARM_TO_SCORE_TARGET_STATE).onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.moveArmToTarget()));
    OP_PAD_BUTTONS.getButton(ButtonCode.RETRACT_ARM_FULL).onTrue(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT));
    
    GAMEPAD.getButton(ButtonCode.A).and((() -> isRobotReadyToScore())).toggleOnTrue(new ScorePieceCommand());
    GAMEPAD.getButton(ButtonCode.X).whileTrue(chargeStationBalancingCommand);

    // When the floor intake button is pressed, update the states
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).and(() -> overallState != OverallState.ENDGAME).onTrue(new InstantCommand(() -> {
      overallState = OverallState.GROUND_PICKUP;
      loadTargetState = LoadTargetState.GROUND;
    }));
    
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER)
    .and(GAMEPAD.getButton(ButtonCode.RIGHTBUMPER))
    .and(GAMEPAD.getButton(ButtonCode.Y))
    .onTrue(new InstantCommand(() -> DRIVE_TRAIN_SUBSYSTEM.zeroGyroscope()));

    new Trigger(() -> GAMEPAD.getAxisIsPressed(AxisCode.RIGHTTRIGGER))
      .whileTrue(new InstantCommand(() -> DRIVE_TRAIN_SUBSYSTEM.cutPower()))
      .onFalse(new InstantCommand(() -> DRIVE_TRAIN_SUBSYSTEM.stopCutPower()));

    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER)
    .and(() -> overallState != OverallState.ENDGAME)
    .toggleOnTrue(new InstantCommand(() -> {
      overallState = OverallState.GROUND_PICKUP;
      loadTargetState = LoadTargetState.GROUND;
    }));

    // When the floor intake button is released, the state needs to be updated:
    //  If it was released without a successful ground pickup, state goes back to empty transit
    //  If it was released AFTER a successful ground pickup, state goes to loaded transit or preparing to score
    //  Pickup target changes to HPS either way
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).toggleOnFalse(new InstantCommand(() -> {
      if (loadState == LoadState.EMPTY) {
        overallState = OverallState.EMPTY_TRANSIT;
      }
      else {
        overallState = OverallState.LOADED_TRANSIT;
      }
      loadTargetState = LoadTargetState.DOUBLE_SUBSTATION;
    }));

    // TODO: bind this to a button
    OP_PAD_SWITCHES.getButton(ButtonCode.DRIVER_CONTROL_OVERRIDE)
    .toggleOnTrue(new InstantCommand(() -> driverControlOverride = true)); 

    OP_PAD_BUTTONS.getButton(ButtonCode.EJECT_PIECE)
    .toggleOnTrue(new InstantCommand(() -> overallState = OverallState.EJECTING));
    
    OP_PAD_SWITCHES.getButton(ButtonCode.ENDGAME_OVERRIDE)
    .toggleOnTrue(new InstantCommand(() -> overallState = OverallState.ENDGAME))
    .toggleOnFalse(new InstantCommand(() -> {
      if (loadState == LoadState.LOADED) {
        overallState = OverallState.LOADED_TRANSIT;
      }
      else {
        overallState = OverallState.EMPTY_TRANSIT;
      }
    }));

    OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_EXTENSION_LIMITS).toggleOnTrue(new InstantCommand(() -> ARM_SUBSYSTEM.enableExtensionLimit(true)));
    OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_EXTENSION_LIMITS).toggleOnFalse(new InstantCommand(() -> ARM_SUBSYSTEM.enableExtensionLimit(false)));
    OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_ROTATION_LIMITS).toggleOnTrue(new InstantCommand(() -> ARM_SUBSYSTEM.enableRotationLimit(true)));
    OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_ROTATION_LIMITS).toggleOnFalse(new InstantCommand(() -> ARM_SUBSYSTEM.enableRotationLimit(false)));
    
    OP_PAD_BUTTONS.getButton(ButtonCode.ROTATE_ARM_BACKWARD)
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.ARM_ROTATION_MANUAL_OVERRIDE))
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_ROTATION_LIMITS))
    .onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.rotateArmBackwardPower()));

    OP_PAD_BUTTONS.getButton(ButtonCode.ROTATE_ARM_FORWARD)
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.ARM_ROTATION_MANUAL_OVERRIDE))
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_ROTATION_LIMITS))
    .onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.rotateArmForwardPower()));

    OP_PAD_BUTTONS.getButton(ButtonCode.EXTEND_ARM)
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.ARM_EXTENSION_MANUAL_OVERRIDE))
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_EXTENSION_LIMITS))
    .onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.extendArmPower()));

    OP_PAD_BUTTONS.getButton(ButtonCode.RETRACT_ARM)
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.ARM_EXTENSION_MANUAL_OVERRIDE))
    .and(OP_PAD_SWITCHES.getButton(ButtonCode.IGNORE_EXTENSION_LIMITS))
    .onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.retractArmPower()));
    
    OP_PAD_SWITCHES.getButton(ButtonCode.LIMELIGHT_LIGHT_OFF_OVERRIDE).onTrue(new InstantCommand(() -> limelightOverride = true)).onFalse(new InstantCommand(() -> limelightOverride = false));
  }

  // Checking for a cone specifically, as opposed to any game piece, is relevant
  // since cones have weight and cubes mostly don't.
  public static boolean hasCone() {
    return loadState == LoadState.LOADED && pieceState == PieceState.CONE;
  }
  
  private void configureTriggers() {
    // CLAW
    // If the claw closes and we have a game piece, we're loaded.
    // new Trigger(() -> Robot.CLAW_SUBSYSTEM.detectsGamePiece() && Robot.clawState == ClawState.CLOSED).onTrue(new InstantCommand(() -> Robot.loadState = LoadState.LOADED));

    // If the claw closes but we do NOT have a game piece, pickup failed, so open the claw again.
    new Trigger(() -> Robot.CLAW_SUBSYSTEM.detectsGamePiece() == false && Robot.clawState == ClawState.CLOSED).onTrue(new InstantCommand(() -> Robot.loadState = LoadState.EMPTY).andThen(new ClawOpenCommand()));
    
    // While the claw is open, check if we detect a game piece, and if we do, close the claw.
    new Trigger(() -> Robot.CLAW_SUBSYSTEM.detectsGamePiece() && Robot.clawState == ClawState.OPEN).onTrue(
      new InstantCommand(() -> Robot.overallState = OverallState.LOADING)
      .andThen(new ClawCloseCommand())
      .andThen(new InstantCommand(() -> Robot.overallState = OverallState.LOADED_TRANSIT))
      .andThen(RUMBLE_COMMAND)
    );
    
    // ARM
    new Trigger(() -> Robot.overallState == OverallState.EJECTING).onTrue(new EjectPieceCommand());

    // // Extend and rotate the arm to the loading target
    // new Trigger(() -> Robot.loadState == LoadState.EMPTY && (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE || Robot.overallState == OverallState.GROUND_PICKUP))
    //     .whileTrue(new ArmRotateToRetrievalPositionCommand(Robot.loadTargetState)
    //     .andThen(new ArmExtendToRetrievalPositionCommand(Robot.loadTargetState)).withName("Extend and rotate arm to loading target"));
    
    // // Extend and rotate the arm to the scoring target
    // new Trigger(() -> Robot.loadState == LoadState.LOADED && Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY)
    //   .onTrue(new ArmRotateToRowPositionCommand(Robot.scoringTargetState)
    //   .andThen(new ArmExtendToRowPositionCommand(Robot.scoringTargetState)).withName("Extend and rotate arm to scoring target"));
    
    // Retract the arm and rotate it upwards if the robot
    // - has just loaded
    // - has just scored
    // - is not in our alliance community or loading zone
    new Trigger(() -> (Robot.overallState == OverallState.LOADED_TRANSIT && Robot.zoneState != ZoneState.ALLIANCE_COMMUNITY) 
                        || (Robot.overallState == OverallState.EMPTY_TRANSIT && Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY)
                        || (Robot.zoneState != ZoneState.ALLIANCE_COMMUNITY && Robot.zoneState != ZoneState.ALLIANCE_LOADING_ZONE))
      .onTrue(new ArmGoToSetpointCommand(Constants.ARM_FULL_RETRACT_SETPOINT).withName("Retract arm"));

    // new Trigger(() -> Robot.hasCone()).whileTrue(RUMBLE_COMMAND);
    // new Trigger(() -> loadState == LoadState.EMPTY && Robot.CLAW_SUBSYSTEM.detectsGamePiece()).onTrue(RUMBLE_COMMAND);
    

    new Trigger(() -> TABLET_SCORING_SUBSYSTEM.GetScoringTarget().GetShape() == ScoringShape.CONE).onTrue(ledsSignalConeCommand.andThen(new InstantCommand(() -> pieceState = PieceState.CONE)));
    new Trigger(() -> TABLET_SCORING_SUBSYSTEM.GetScoringTarget().GetShape() == ScoringShape.CUBE).onTrue(ledsSignalCubeCommand.andThen(new InstantCommand(() -> pieceState = PieceState.CUBE)));
  
    new Trigger(() -> TABLET_SCORING_SUBSYSTEM.GetScoringTarget().GetPosition().RowEquals(2)).onTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.LOW));
    new Trigger(() -> TABLET_SCORING_SUBSYSTEM.GetScoringTarget().GetPosition().RowEquals(1)).onTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.MID));
    new Trigger(() -> TABLET_SCORING_SUBSYSTEM.GetScoringTarget().GetPosition().RowEquals(0)).onTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.HIGH));



  }

  private void setZoneStateFromFieldZone() {
    FieldZone fieldZone = fieldSubzone.getFieldZone();
    boolean allianceOwnsZone = allianceColor == fieldZone.getZoneOwner();

    switch (fieldZone.getMacroZone()) {
      case NONE:
        zoneState = ZoneState.NONE;
        break;
      case NEUTRAL:
          zoneState = ZoneState.NEUTRAL;
          break;
      case LOADING_ZONE:
        if (allianceOwnsZone) {
          zoneState = ZoneState.ALLIANCE_LOADING_ZONE;
        }
        else {
          zoneState = ZoneState.OPPONENT_LOADING_ZONE;
        }
        break;
      case COMMUNITY:
        if (allianceOwnsZone) {
          zoneState = ZoneState.ALLIANCE_COMMUNITY;
        }
        else {
          zoneState = ZoneState.OPPONENT_COMMUNITY;
        }
        break;
      case CHARGE_STATION:
        if (allianceOwnsZone) {
          zoneState = ZoneState.ALLIANCE_CHARGE_STATION;
        }
        else {
          zoneState = ZoneState.OPPONENT_CHARGE_STATION;
        }
        break;
    }
  }

  private boolean isRobotReadyToScore() {
    // TODO: implement this method

    // Components:
    // Is robot in correct location
    //    This could be either the proper pose,
    //    or the correct angle if we end up using the tape
    // Is arm in correct configuration (rotation and orientation)
    return true;
  }

  // Due to the manner in which the robot connects to the driver station,
  // which differs between the shop and match play,
  // this method needs to called both periodically AND in the auto/tele init methods.
  private void setDriverStationData() {
    if (allianceColor == Alliance.Invalid) {
      allianceColor = DriverStation.getAlliance();
    }
  }

  public boolean getLimelightOverride() {
    return limelightOverride;
  }
}
