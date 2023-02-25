package frc.robot.commands.groups;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.AdjustArmToRetrievalPosition;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.util.StateManagementNew.ArmRotationState;
import frc.util.StateManagementNew.OverallState;
import frc.util.StateManagementNew.PieceState;
import frc.util.StateManagementNew.ScoringTargetState;

public class EjectPieceCommand extends SequentialCommandGroup {
    public EjectPieceCommand() {
        addCommands(
            new InstantCommand(() -> Robot.overallState = OverallState.EJECTING),
            new AdjustArmToRetrievalPosition(ArmRotationState.COLLECT_GROUND),
            new ExtendArmCommand(),
            new OpenClawCommand(),
            new WaitCommand(0.25),
            new RetractArmCommand(),
            new InstantCommand(() -> Robot.overallState = OverallState.EMPTY_TRANSIT),
            new InstantCommand(() -> Robot.pieceState = PieceState.NONE),
            new InstantCommand(() -> Robot.scoringTargetState = ScoringTargetState.NONE)
        );
    }
}
