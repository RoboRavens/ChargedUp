// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSequencedRetractionFromReverseCommand extends SequentialCommandGroup {
  /** Creates a new ArmSequencedRetractionCommand. */
  public ArmSequencedRetractionFromReverseCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmGoToSetpointDangerousRetractionOnlyCommand(Constants.ARM_SEQUENCED_RETRACTION_FROM_REVERSE_STAGING_SETPOINT, 1.25),
      // new WaitCommand(1),
      new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT)
    );
  }
}
