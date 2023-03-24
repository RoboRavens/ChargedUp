package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LEDs.LEDsSolidColorCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.util.Colors;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.PieceState;
import frc.util.StateManagement.ScoringTargetState;

public class ScorePieceCommand extends SequentialCommandGroup {
    public ScorePieceCommand() {
        // This command does not operate upon the drivetrain,
        // but requiring it will lock out movement during scoring, which is probably wise.
        addRequirements(Robot.CLAW_SUBSYSTEM, Robot.ARM_SUBSYSTEM, Robot.DRIVE_TRAIN_SUBSYSTEM);
        addCommands(
            new InstantCommand(() -> Robot.overallState = OverallState.SCORING),
            new ClawOpenCommand(),
            new ParallelCommandGroup(
                new InstantCommand(() -> Robot.overallState = OverallState.EMPTY_TRANSIT),
                new InstantCommand(() -> Robot.pieceState = PieceState.NONE),
                new InstantCommand(() -> Robot.scoringTargetState = ScoringTargetState.NONE),
                new LEDsSolidColorCommand(Robot.LED_SUBSYSTEM, Colors.LIGHT_GREEN)
            )
        );
    }
}
