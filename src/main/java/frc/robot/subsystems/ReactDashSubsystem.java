package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactDashSubsystem extends SubsystemBase {
  public static final NetworkTable ReactDash = NetworkTableInstance.getDefault().getTable("ReactDash");
  public static final String AUTO_TAB_NAME = "Autonomous";
  public static final String TELEOP_TAB_NAME = "Teleop";

  private StringPublisher _goTotabPub;
  private IntegerPublisher _locationPub;
  
  private BooleanPublisher _joystick0Pub;
  private BooleanPublisher _joystick2Pub;
  private BooleanPublisher _joystick3Pub;

  public ReactDashSubsystem() {
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Main");
    _goTotabPub = autoTable.getStringTopic("rpub/goTotab").publish();
    _locationPub = autoTable.getIntegerTopic("rpub/driverStation").publish();

    _joystick0Pub = autoTable.getBooleanTopic("rpub/joystick0").publish();
    _joystick2Pub = autoTable.getBooleanTopic("rpub/joystick2").publish();
    _joystick3Pub = autoTable.getBooleanTopic("rpub/joystick3").publish();
  }

  @Override
  public void periodic() {
    _locationPub.set(DriverStation.getLocation());
    
    _joystick0Pub.set(DriverStation.getStickButtonCount(0) > 0);
    _joystick2Pub.set(DriverStation.getStickButtonCount(2) > 0);
    _joystick3Pub.set(DriverStation.getStickButtonCount(3) > 0);
  }

  public void SwitchTab(String tab) {
    _goTotabPub.set(tab);
  }
}
