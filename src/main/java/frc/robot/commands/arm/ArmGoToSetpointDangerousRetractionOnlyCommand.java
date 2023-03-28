// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.util.arm.ArmMaximumConstraint;
import frc.util.arm.ArmPose;
import frc.util.arm.ArmSetpoint;

public class ArmGoToSetpointDangerousRetractionOnlyCommand extends CommandBase {
  private ArrayList<ArmSetpoint> subSetpoints = new ArrayList<ArmSetpoint>();
  private ArmSubsystem arm = Robot.ARM_SUBSYSTEM;
  private ArmSetpoint finalSetpoint;
  private int setpointIterator = 0;
  private Timer timer = new Timer();
  private double _timeoutSeconds = 0;
  private ArmSetpoint currentSubSetpoint;

  public ArmGoToSetpointDangerousRetractionOnlyCommand(ArmSetpoint setpoint, double timeoutSeconds) {
    this.finalSetpoint = setpoint;
    this.currentSubSetpoint = setpoint;
    this._timeoutSeconds = timeoutSeconds;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    //System.out.println("TIMER: " + timer.get());
    // If the arm is extended further than the setpoint given,
    // replace the setpoint with the current arm extension.
    // This command is to retract the arm only, NEVER to extend it.
    
    //SmartDashboard.putNumber("Final Ext Starting", finalSetpoint.getExtensionSetpoint());
    //SmartDashboard.putNumber("Current Ext Starting", arm.getCurrentExtensionNativeUnits());
    
    
    finalSetpoint.setExtensionSetpoint(Math.min(finalSetpoint.getExtensionSetpoint(), arm.getCurrentExtensionNativeUnits()));
  
    //SmartDashboard.putNumber("New Final Ext", finalSetpoint.getExtensionSetpoint());
    

    //SmartDashboard.putNumber("FS get rotation SP", finalSetpoint.getRotationSetpoint());

//    SmartDashboard.putNumber("Current rotation NU", arm.getCurrentRotationNativeUnits());

    // Same with rotation.
    // This logic will need to be updated to handle setpoints on the other side of 0,
    // which could have a lower absolute value but cause the arm to rotate significantly.
    // Right now it is only to be used to retract on the same side of 0.
    
    finalSetpoint.setRotationSetpoint(Math.min(finalSetpoint.getRotationSetpoint(), arm.getCurrentRotationNativeUnits()));
  
    
    // if (Math.abs(finalSetpoint.getRotationSetpoint()) > Math.abs(arm.getCurrentRotationNativeUnits())) {
    //  finalSetpoint.setRotationSetpoint(arm.getCurrentRotationNativeUnits());
    // }
    
//    finalSetpoint.setRotationSetpoint(_timeoutSeconds);

    
    //SmartDashboard.putNumber("New FS Rotation", finalSetpoint.getRotationSetpoint());


    arm.setFinalRotationSetpoint(finalSetpoint.getRotationSetpoint());
    arm.setFinalExtensionSetpoint(finalSetpoint.getExtensionSetpoint());
    arm.brakeDisable();

    arm.motionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putString("ArmSetpoint", finalSetpoint.getName());
    //SmartDashboard.putNumber("Arm Rotation Target", finalSetpoint.getRotationSetpointDegrees());
    
    //SmartDashboard.putNumber("Arm Extension Target", finalSetpoint.getExtensionSetpoint());
    
    arm.brakeDisable();

    arm.setArmRotationPosition(finalSetpoint.getRotationSetpoint(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    arm.setArmExtensionPosition(finalSetpoint.getExtensionSetpoint(), Constants.ARM_EXTENSION_VELOCITY, Constants.ARM_EXTENSION_ACCELERATION);
  }

  private boolean setpointIsFinished(ArmSetpoint setpoint) {
    double rotationError = Math.abs(setpoint.getRotationSetpoint() - arm.getCurrentRotationNativeUnits());
    double extensionError = Math.abs(setpoint.getExtensionSetpoint() - arm.getCurrentExtensionNativeUnits());

    boolean rotationIsFinished = rotationError < Constants.ARM_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;
    boolean extensionIsFinished = extensionError < Constants.ARM_EXTENSION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;

    /*
    SmartDashboard.putBoolean("EXT FIN", extensionIsFinished);
    SmartDashboard.putNumber("EXT error", extensionError);

    SmartDashboard.putBoolean("ROT FIN", rotationIsFinished);
    SmartDashboard.putNumber("ROT error", rotationError);
*/

    return rotationIsFinished && extensionIsFinished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Note that terminating this command early, for example due to a timeout,
    // does not change the target of the arm, so if the next command issued
    // continues moving the arm to its target, it will do so.
    // return setpointIsFinished(finalSetpoint) || timer.hasElapsed(_timeoutSeconds);
    return timer.hasElapsed(_timeoutSeconds);
  }
}
