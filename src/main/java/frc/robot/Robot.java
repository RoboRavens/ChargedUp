// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.robot.commands.arm.AdjustArmToRetrievalPosition;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystemBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.util.StateManagementNew;
import frc.util.StateManagementNew.ArmExtensionState;
import frc.util.StateManagementNew.ArmRotationState;
import frc.util.StateManagementNew.ClawState;
import frc.util.StateManagementNew.DrivetrainState;
import frc.util.StateManagementNew.LimelightState;
import frc.util.StateManagementNew.LoadState;
import frc.util.StateManagementNew.LoadTargetState;
import frc.util.StateManagementNew.OverallState;
import frc.util.StateManagementNew.PieceState;
import frc.util.StateManagementNew.ScoringTargetState;
// import frc.util.StateManagement;
import frc.util.scoring_states.GamePieceState;
import frc.util.scoring_states.PieceRetrievalState;
import frc.util.scoring_states.RowSelectionState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static final DrivetrainSubsystemBase DRIVE_TRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  public static final Joystick JOYSTICK = new Joystick(0);
  public static final Gamepad GAMEPAD = new Gamepad(JOYSTICK);
  public static final Gamepad OP_PAD = new Gamepad(1);
  public static final ChargeStationBalancingCommand chargeStationBalancingCommand = new ChargeStationBalancingCommand();
  // public static GamePieceState gamePieceState = GamePieceState.CLEAR;
  // public static RowSelectionState rowSelectionState = RowSelectionState.CLEAR;
  // public static PieceRetrievalState pieceRetrievalState = PieceRetrievalState.CLEAR;
  public static final ArmSubsystem ARM_SUBSYSTEM = new ArmSubsystem();
  public static final ClawSubsystem CLAW_SUBSYSTEM = new ClawSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  // public static final StateManagement STATE_MANAGEMENT = new StateManagement();
  public static boolean driverControlOverride = false;

  // Sets the default robot mechanism states (may need to be changed)
  public static OverallState overallState = OverallState.EMPTY_TRANSIT;
  public static ArmRotationState armRotation = ArmRotationState.COLLECT_GROUND;
  public static ArmExtensionState armExtensionState = ArmExtensionState.RETRACTED;
  public static PieceState pieceState = PieceState.NONE;
  public static LoadState loadState = LoadState.EMPTY;
  public static LoadTargetState loadTargetState = LoadTargetState.GROUND;
  public static DrivetrainState drivetrainState = DrivetrainState.FREEHAND;
  public static ClawState clawState = ClawState.CLOSED;
  public static LimelightState limelightState = LimelightState.TAG_TRACKING;
  public static ScoringTargetState scoringTargetState = ScoringTargetState.NONE;

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Odometry rotation (degrees)", DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getDegrees());
    SmartDashboard.putNumber("Gyroscope rotation (degrees)", DRIVE_TRAIN_SUBSYSTEM.getGyroscopeRotation2dTest().getDegrees());
    // STATE_MANAGEMENT.manageStates();
    setOverallStates();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
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

  private void configureButtonBindings() {
    // AxisCode.LEFTTRIGGER and ButtonCode.B are being used in the StateManagement class
    // GAMEPAD.getButton(ButtonCode.A).whileTrue(chargeStationBalancingCommand);
    // OP_PAD.getButton(ButtonCode.CUBE).toggleOnTrue(new InstantCommand(() -> gamePieceState = GamePieceState.CUBE));
    // OP_PAD.getButton(ButtonCode.CONE).toggleOnTrue(new InstantCommand(() -> gamePieceState = GamePieceState.CONE));
    // OP_PAD.getButton(ButtonCode.SCORE_LOW).toggleOnTrue(new InstantCommand(() -> rowSelectionState = RowSelectionState.LOW));
    // OP_PAD.getButton(ButtonCode.SCORE_MID).toggleOnTrue(new InstantCommand(() -> rowSelectionState = RowSelectionState.MID));
    // OP_PAD.getButton(ButtonCode.SCORE_HIGH).toggleOnTrue(new InstantCommand(() -> rowSelectionState = RowSelectionState.HIGH));
    // OP_PAD.getButton(ButtonCode.SUBSTATION_INTAKE).toggleOnTrue(new InstantCommand(() -> pieceRetrievalState = PieceRetrievalState.SUBSTATION));
    // OP_PAD.getButton(ButtonCode.FLOOR_INTAKE).toggleOnTrue(new InstantCommand(() -> pieceRetrievalState = PieceRetrievalState.FLOOR));
    // OP_PAD.getButton(ButtonCode.DRIVER_CONTROL_OVERRIDE).toggleOnTrue(new InstantCommand(() -> driverControlOverride = true)); // May not toggle as intended depending on the button type on the panel (i.e. button vs switch)
    // Maybe include a button to clear all states?

    GAMEPAD.getButton(ButtonCode.A).whileTrue(chargeStationBalancingCommand);
    OP_PAD.getButton(ButtonCode.CUBE).toggleOnTrue(new InstantCommand(() -> pieceState = PieceState.CUBE));
    OP_PAD.getButton(ButtonCode.CONE).toggleOnTrue(new InstantCommand(() -> pieceState = PieceState.CONE));
    OP_PAD.getButton(ButtonCode.SCORE_LOW).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.LOW));
    OP_PAD.getButton(ButtonCode.SCORE_MID).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.MID));
    OP_PAD.getButton(ButtonCode.SCORE_HIGH).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.HIGH));
    OP_PAD.getButton(ButtonCode.SUBSTATION_INTAKE).toggleOnTrue(new InstantCommand(() -> loadTargetState = LoadTargetState.HPS));
    OP_PAD.getButton(ButtonCode.FLOOR_INTAKE).toggleOnTrue(new InstantCommand(() -> loadTargetState = LoadTargetState.GROUND));
    OP_PAD.getButton(ButtonCode.DRIVER_CONTROL_OVERRIDE).toggleOnTrue(new InstantCommand(() -> driverControlOverride = true)); // May not toggle as intended depending on the button type on the panel (i.e. button vs switch)
    OP_PAD.getButton(ButtonCode.EJECT_PIECE).toggleOnTrue(new InstantCommand(() -> overallState = OverallState.EJECTING).andThen(new OpenClawCommand()).andThen(new InstantCommand(() -> overallState = OverallState.EMPTY_TRANSIT)));
  }

  private void setOverallStates() {
    if (overallState != OverallState.EJECTING) {
      if (loadState == LoadState.LOADED && LIMELIGHT_SUBSYSTEM.isInAllianceCommunity()) {
        if (drivetrainState == DrivetrainState.FINAL_SCORING_ROTATION_LOCK_AND_AUTO_ALIGN) {
          overallState = OverallState.FINAL_SCORING_ALIGNMENT;
        }
        else if (drivetrainState == DrivetrainState.SCORING && armExtensionState == ArmExtensionState.ACTIVELY_SCORING) {
          overallState = OverallState.SCORING;
        }
        else {
          overallState = OverallState.PREPARING_TO_SCORE;
        }
      }
      else if (loadState == LoadState.LOADED) {
        overallState = OverallState.LOADED_TRANSIT;
      }
      else if (loadState == LoadState.EMPTY) {
        if (armExtensionState == ArmExtensionState.COLLECT_GROUND) {
          overallState = OverallState.GROUND_PICKUP;
        }
        else if (clawState == ClawState.OPEN && CLAW_SUBSYSTEM.detectsGamePiece()) {
          overallState = OverallState.LOADING;
        }
        else {
          overallState = OverallState.EMPTY_TRANSIT;
        }
      }
    } 
  }
}
