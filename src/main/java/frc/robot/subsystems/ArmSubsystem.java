package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.arm.AdjustArmToRowPosition;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.util.StateManagementNew.OverallState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    @Override
    public void periodic() {
        if (Robot.overallState == OverallState.PREPARING_TO_SCORE) {
            new AdjustArmToRowPosition(Robot.scoringTargetState).andThen(new ExtendArmCommand()).schedule();
        }
    }
}
