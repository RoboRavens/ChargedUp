package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    @Override
    public void periodic() {
        setAndManageArmStates();
    }

    private void setAndManageArmStates() {
        // TODO: Implement this command
        // Set the arm states and schedule arm rotation/extension commands based on overall state of the robot
    }
}
