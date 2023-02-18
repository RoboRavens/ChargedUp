package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.arm.AdjustArmToRowPosition;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.util.StateManagementNew.ArmExtensionState;
import frc.util.StateManagementNew.ArmRotationState;
import frc.util.StateManagementNew.OverallState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    @Override
    public void periodic() {
        setArmRotationState();
        setArmExtensionState();
    }

    private void setArmRotationState() {
        // TODO: set Robot.armRotationState
        // This should reflect the actual state of the arm (its current rotation), NOT what the driver intends to do with the arm
    }

    private void setArmExtensionState() {
        // TODO: set Robot.armExtensionState
        // This should reflect the actual state of the arm (its current position), NOT what the driver intends to do with the arm
    }

}
