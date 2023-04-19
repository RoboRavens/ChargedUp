package frc.robot.subsystems.TabletScoring;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ReactDashSubsystem;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.field.FieldMeasurements;

public class TeleopSubystem extends SubsystemBase {
  
  private StringSubscriber _columnSub;
  private StringSubscriber _rowSub;
  private StringSubscriber _shapeSub;
  private StringSubscriber _substationSub;

  private StringEntry _columnPub;
  private StringEntry _rowPub;
  private StringEntry _shapePub;
  private StringEntry _substationPub;

  private Timer _lockTimer = new Timer();

  public TeleopSubystem(){
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _columnSub = autoTable.getStringTopic("dpub/column").subscribe("-1");
    _rowSub = autoTable.getStringTopic("dpub/row").subscribe("-1");
    _shapeSub = autoTable.getStringTopic("dpub/shape").subscribe("NONE");
    _substationSub = autoTable.getStringTopic("dpub/substation").subscribe("NONE");

    _columnPub = autoTable.getStringTopic("rpub/column").getEntry("-1");
    _rowPub = autoTable.getStringTopic("rpub/row").getEntry("-1");
    _shapePub = autoTable.getStringTopic("rpub/shape").getEntry("NONE");
    _substationPub = autoTable.getStringTopic("rpub/substation").getEntry("NONE");

    _lockTimer.start();
  }

  @Override
  public void periodic(){
    if (_lockTimer.get() > .5) {
      _columnPub.set(_columnSub.get("-1"));
      _rowPub.set(_rowSub.get("-1"));
      _shapePub.set(_shapeSub.get("NONE"));;
    }

    _substationPub.set(_substationSub.get("SINGLE"));
  }

  public void ShowTab() {
    Robot.REACT_DASH_SUBSYSTEM.SwitchTab(ReactDashSubsystem.TELEOP_TAB_NAME);
  }

  public void ClearSelectedPositionAndShape(){
    _lockTimer.reset();
    _lockTimer.start();
    _columnPub.set("-1");
    _rowPub.set("-1");
    _shapePub.set("NONE");
  }

  public boolean isCubeColumn() {
    var column = this.GetScoringColumn();
    switch(column) {
      case 1:
      case 4:
      case 7:
      return true;
      default:
      return false;
    }
  }

  private int GetScoringColumn() {
    try {
      return Integer.parseInt(_columnPub.get("-1"));
    } catch(NumberFormatException e) {
      return -1;
    }
  }

  private int GetScoringRow() {
    try {
      return Integer.parseInt(_rowPub.get("-1"));
    } catch(NumberFormatException e) {
      return -1;
    }
  }

  public ScoringTargetState GetScoringState() {
    switch (this.GetScoringRow()) {
      case 0:
      return ScoringTargetState.HIGH;
      case 1:
      return ScoringTargetState.MID;
      case 2:
      return ScoringTargetState.LOW;
      default:
      return ScoringTargetState.NONE;
    }
  }

  public Translation2d GetScoringCoordinates() {
    var column = this.GetScoringColumn();
    var row = this.GetScoringRow();
    if (column == -1 || row == -1) {
      return null;
    }

    if (Robot.allianceColor == Alliance.Invalid) {
      return null;
    }
    
    return Robot.allianceColor == Alliance.Blue ?
      FieldMeasurements.BLUE_NODE_COORD[column] : FieldMeasurements.RED_NODE_COORD[column];
  }

  public ScoringShape GetScoringShape() {
    switch(_shapeSub.get("NONE")) {
      case "CUBE":
      return ScoringShape.CUBE;
      case "CONE":
      return ScoringShape.CONE;
      default:
      return ScoringShape.NONE;
    }
  }

  public Substation GetSubstation() {
    switch(_substationSub.get("NONE")) {
      case "SINGLE":
      return Substation.SINGLE;
      case "DOUBLE_LEFT":
      return Substation.DOUBLE_LEFT;
      case "DOUBLE_RIGHT":
      return Substation.DOUBLE_RIGHT;
      default:
      return Substation.NONE;
    }
  }

  public Translation2d GetSubstationCoordinates() {
    var substation = this.GetSubstation();
    if (substation == Substation.NONE) {
      return null;
    }

    if (Robot.allianceColor == Alliance.Invalid) {
      return null;
    }

    return Robot.allianceColor == Alliance.Blue ?
      FieldMeasurements.BLUE_SUBSTATIONS[substation.Index] : FieldMeasurements.RED_SUBSTATIONS[substation.Index];
  }
}
