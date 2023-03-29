package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactBoardSubsystem extends SubsystemBase {
  public static final NetworkTable ReactDash = NetworkTableInstance.getDefault().getTable("ReactDash");

  public ReactBoardSubsystem() {
  }

  @Override
  public void periodic() {
    
  }
}
