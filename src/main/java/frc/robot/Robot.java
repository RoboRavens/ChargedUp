// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.groups.ScorePieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystemBase;
import frc.robot.subsystems.LimelightSubsystem;
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
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightTrajectorySubsystem;
import frc.robot.subsystems.TrajectoryTestingSubsystem;
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
  public static final Gamepad OP_PAD = new Gamepad(1);
  public static final Gamepad OP_PAD_BUTTONS = new Gamepad(2);
  public static final Gamepad OP_PAD_SWITCHES = new Gamepad(3);
  public static final LimelightTrajectorySubsystem LIMELIGHT_TRAJECTORY_SUBSYSTEM = new LimelightTrajectorySubsystem();
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
  public static ArmRotationState armRotationState = ArmRotationState.COLLECT_GROUND;
  public static ArmExtensionState armExtensionState = ArmExtensionState.RETRACTED;
  public static PieceState pieceState = PieceState.NONE;
  public static LoadState loadState = LoadState.EMPTY;
  public static LoadTargetState loadTargetState = LoadTargetState.HPS;
  public static DrivetrainState drivetrainState = DrivetrainState.FREEHAND;
  public static ClawState clawState = ClawState.CLOSED;
  public static LimelightState limelightState = LimelightState.TAG_TRACKING;
  public static ScoringTargetState scoringTargetState = ScoringTargetState.NONE;
  public static ZoneState zoneState = ZoneState.NEUTRAL;

  public static FieldZones fieldZones = new FieldZones();
  public static FieldSubzone fieldSubzone = FieldZones.noneSubzone;

  
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

    GAMEPAD.getButton(ButtonCode.B).onTrue(new InstantCommand(() -> Robot.LIMELIGHT_TRAJECTORY_SUBSYSTEM.driveTrajectory()));
    GAMEPAD.getButton(ButtonCode.B).onTrue(new InstantCommand(() -> Robot.LIMELIGHT_TRAJECTORY_SUBSYSTEM.goToScoringPosition()));
    ARM_SUBSYSTEM.setAndManageArmStates();
    CLAW_SUBSYSTEM.setAndManageClawStates();
    // Temp button bindings to simulate zone and claw state

    OP_PAD_BUTTONS.getButton(ButtonCode.TEMP_ALLIANCE_LOADING_ZONE).onTrue(new InstantCommand(() -> zoneState = ZoneState.ALLIANCE_LOADING_ZONE));
    OP_PAD_BUTTONS.getButton(ButtonCode.TEMP_ALLIANCE_COMMUNITY_ZONE).onTrue(new InstantCommand(() -> zoneState = ZoneState.ALLIANCE_COMMUNITY));
    OP_PAD_BUTTONS.getButton(ButtonCode.TEMP_NEUTRAL_ZONE).onTrue(new InstantCommand(() -> zoneState = ZoneState.NEUTRAL));
    OP_PAD_BUTTONS.getButton(ButtonCode.TEMP_OPPONENT_ZONES).onTrue(new InstantCommand(() -> zoneState = ZoneState.OPPONENT_COMMUNITY));
    OP_PAD_SWITCHES.getButton(ButtonCode.TEMP_IS_LOADED).toggleOnTrue(new InstantCommand(() -> {loadState = LoadState.LOADED; overallState = OverallState.LOADED_TRANSIT;}));
    OP_PAD_SWITCHES.getButton(ButtonCode.TEMP_IS_LOADED).toggleOnFalse(new InstantCommand(() -> {loadState = LoadState.EMPTY; overallState = OverallState.EMPTY_TRANSIT;}));
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

  private void setNonButtonDependentOverallStates() {
    // If the left trigger is pressed,
    // Set the overall state to either scoring alignment or hps pickup based on the zone state
    if (GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
      if (zoneState == ZoneState.ALLIANCE_COMMUNITY) {
        overallState = OverallState.FINAL_SCORING_ALIGNMENT;
      }
      else if (zoneState == ZoneState.ALLIANCE_LOADING_ZONE) {
        Robot.drivetrainState = DrivetrainState.HPS_ALIGN;
      }
    }
    // Changes the overall state to empty or loaded transit when the trigger is released
    else if (Robot.overallState == OverallState.FINAL_SCORING_ALIGNMENT || drivetrainState == DrivetrainState.HPS_ALIGN) {
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
    // Sets the robot state to HPS_PICKUP only once when the robot does not have a piece and is in the alliance loading zone
    if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.overallState == OverallState.EMPTY_TRANSIT) {
      Robot.overallState = OverallState.HPS_PICKUP;
    }
  }

  private void configureButtonBindings() {
    // Driver controller
    // If the release button is pressed and the robot is aligned with a scoring node, score the piece
    GAMEPAD.getButton(ButtonCode.Y).and((() -> isRobotReadyToScore())).toggleOnTrue(new ScorePieceCommand());
    // Balance on the charge station while A is pressed
    GAMEPAD.getButton(ButtonCode.A).whileTrue(chargeStationBalancingCommand);
    // When the floor intake button is pressed, update the states
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).toggleOnTrue(new InstantCommand(() -> {
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
      loadTargetState = LoadTargetState.HPS;
    }));
    // GAMEPAD.getButton(ButtonCode.LEFTSTICK).whileTrue(new OpenClawCommand);

    // This should likely be part of driver control as well, but have to think about which button...
    GAMEPAD.getButton(ButtonCode.DRIVER_CONTROL_OVERRIDE).toggleOnTrue(new InstantCommand(() -> driverControlOverride = true)); // May not toggle as intended depending on the button type on the panel (i.e. button vs switch)

    OP_PAD.getButton(ButtonCode.CUBE).toggleOnTrue(new InstantCommand(() -> pieceState = PieceState.CUBE));
    OP_PAD.getButton(ButtonCode.CONE).toggleOnTrue(new InstantCommand(() -> pieceState = PieceState.CONE));
    OP_PAD.getButton(ButtonCode.SCORE_LOW).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.LOW));
    OP_PAD.getButton(ButtonCode.SCORE_MID).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.MID));
    OP_PAD.getButton(ButtonCode.SCORE_HIGH).toggleOnTrue(new InstantCommand(() -> scoringTargetState = ScoringTargetState.HIGH));
    OP_PAD.getButton(ButtonCode.SUBSTATION_INTAKE).toggleOnTrue(new InstantCommand(() -> loadTargetState = LoadTargetState.HPS));

    // This looks correct to me but the ejection process is likely a bit more complicated than simply opening the claw; if the piece falls in the wrong spot, it could be bad.
    // We probably want a command group that handles piece ejection. This could include rotating the robot and/or the arm to ensure the piece falls neither inside the robot,
    // or directly in front of the drivetrain.
    // The EJECTING state is handled in the arm subsystem
    OP_PAD.getButton(ButtonCode.EJECT_PIECE).toggleOnTrue(new InstantCommand(() -> overallState = OverallState.EJECTING));
  }

  private boolean isRobotReadyToScore() {
    // TODO: implement this method
    return true;
  }
}
