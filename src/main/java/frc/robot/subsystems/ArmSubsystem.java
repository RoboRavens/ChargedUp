package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.arm.RotateArmToRetrievalPositionCommand;
import frc.robot.commands.arm.RotateArmToRowPositionCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.arm.ExtendArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRowPositionCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.util.StateManagementNew.ArmExtensionState;
import frc.util.StateManagementNew.LoadState;
import frc.util.StateManagementNew.LoadTargetState;
import frc.util.StateManagementNew.OverallState;
import frc.util.StateManagementNew.ZoneState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    @Override
    public void periodic() {
        setAndManageArmStates();
    }

    private void setAndManageArmStates() {
        // Schedules commands that require the arm subsystem
        // The commands scheduled set the arm state in turn
        if (Robot.overallState == OverallState.EJECTING) {
            new EjectPieceCommand().schedule();
        }
        else if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE || Robot.overallState == OverallState.GROUND_PICKUP) {
            // Extend and rotate the arm to the loading target
            new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)
            .andThen(new ExtendArmToRetrievalPositionCommand(Robot.loadTargetState)).schedule();
        }
        else if (Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY) {
            // Extend and rotate the arm to the scoring target
            new RotateArmToRowPositionCommand(Robot.scoringTargetState)
            .andThen(new ExtendArmToRowPositionCommand(Robot.scoringTargetState)).schedule();
        }
        // (When the robot is either in the neutral area, any of the opposite alliance's zones, or on our alliance bridge)
        else {
            // Retract the arm, but rotate the arm to the scoring/loading position
            if (Robot.loadState == LoadState.LOADED) {
                new RetractArmCommand().andThen(new RotateArmToRowPositionCommand(Robot.scoringTargetState)).schedule();
            }
            else {
                new RetractArmCommand().andThen(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)).schedule();
            }
        }
    }
}
