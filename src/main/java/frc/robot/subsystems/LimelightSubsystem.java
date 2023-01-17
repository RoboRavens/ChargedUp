package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem {
    
   LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tv = table.getEntry("tv");
    int camMode = 0;
    
   /*  public double limelightPipelineConfiguration() {

      if (PIPELINEBUTTON0ISPRESSED) {
        int pipeline = 0;
     }

      if (PIPELINEBUTTON1ISPRESSED) {
        int pipeline = 1;
     }

      if (pipeline = 1 & LEFTTRIGGERISPULLED) {
        int ledMode = 3;
     }
   }
    */
   public double getTx() {
    return tx.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

}




