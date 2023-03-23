// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultCommand extends CommandBase {
  private ArmSubsystem arm = Robot.ARM_SUBSYSTEM;
  
  public ArmDefaultCommand() {
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For each degree of freedom, if the override is off, hold the arm in that position.
    // If the override is on, do nothing.
    if (Robot.ARM_ROTATION_MANUAL_OVERRIDE == false) {
      arm.setArmRotationPosition(arm.getArmRotationFinalTargetNativeUnits(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    }
    else {
      arm.stopRotation();
    }

    if (Robot.ARM_EXTENSION_MANUAL_OVERRIDE == false) {
      arm.setArmExtensionPosition(arm.getArmExtensionFinalTargetNativeUnits(), Constants.ARM_EXTENSION_VELOCITY, Constants.ARM_EXTENSION_ACCELERATION);
    }
    else {
  
      arm.stopExtension();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
