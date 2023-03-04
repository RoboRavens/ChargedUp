package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.arm.RotateArmToRetrievalPositionCommand;
import frc.robot.commands.arm.RotateArmToRowPositionCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.arm.ExtendArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRowPositionCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.ZoneState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    public ArmSubsystem() {
        setAndManageArmStates();
    }

    @Override
    public void periodic() {}

    private void setAndManageArmStates() {
        // Schedules commands that require the arm subsystem
        // The commands scheduled set the arm state in turn
        new Trigger(() -> Robot.overallState == OverallState.EJECTING).whileTrue(new EjectPieceCommand());
        // Extend and rotate the arm to the loading target
        new Trigger(() -> (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE || Robot.overallState == OverallState.GROUND_PICKUP) && Robot.loadState == LoadState.EMPTY)
        .whileTrue(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)
        .andThen(new ExtendArmToRetrievalPositionCommand(Robot.loadTargetState)));
        // Extend and rotate the arm to the scoring target
        new Trigger(() -> Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY && Robot.loadState == LoadState.LOADED)
        .whileTrue(new RotateArmToRowPositionCommand(Robot.scoringTargetState)
        .andThen(new ExtendArmToRowPositionCommand(Robot.scoringTargetState)));
        // Retract the arm and rotate it upwards if it is not in the alliance community or alliance loading zone
        new Trigger(() -> Robot.overallState == OverallState.LOADED_TRANSIT || Robot.overallState == OverallState.EMPTY_TRANSIT)
        .whileTrue(new RetractArmCommand().andThen(new RotateArmToRowPositionCommand(ScoringTargetState.HIGH)));
    }
}
