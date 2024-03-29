// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsSubsystem;

public class LEDsAlternatingColorsCommand extends CommandBase {
  LEDsSubsystem leds;
  /** Creates a new LEDsBlinkingCommand. */
  public LEDsAlternatingColorsCommand(LEDsSubsystem subsystem) {
    leds = subsystem;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.ledsAlternatingColors(255, 0, 0, 0, 0, 255);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
