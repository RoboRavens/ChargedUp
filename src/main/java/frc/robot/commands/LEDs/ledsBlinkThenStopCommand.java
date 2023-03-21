// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsSubsystem;
import frc.util.Color;

public class LEDsBlinkThenStopCommand extends CommandBase {
  LEDsSubsystem leds;
  int red = 0;
  int green = 0;
  int blue = 0;

  /** Creates a new DefualtLEDs. */
  public LEDsBlinkThenStopCommand(LEDsSubsystem subsystem, Color color) {
    this.red = color.getRed();
    this.green = color.getGreen();
    this.blue = color.getBlue();
    leds = subsystem;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.ledsBlinkThenStop(red, green, blue, 0, 0, 0);
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
