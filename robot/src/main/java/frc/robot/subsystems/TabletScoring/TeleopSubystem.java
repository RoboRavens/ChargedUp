package frc.robot.subsystems.TabletScoring;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
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
import frc.util.field.FieldSubzone;
import frc.util.field.FieldZone;
import frc.util.field.FieldZones;

public class TeleopSubystem extends SubsystemBase {
  
  private StringSubscriber _columnSub;
  private StringSubscriber _rowSub;
  private StringSubscriber _shapeSub;
  private StringSubscriber _substationSub;
  private DoubleSubscriber _yCoordinateSub;
  private DoubleSubscriber _xCoordinateSub;
  private DoubleArraySubscriber _fieldZonesSub;

  private StringEntry _columnPub;
  private StringEntry _rowPub;
  private StringEntry _shapePub;
  private StringEntry _substationPub;
  private DoubleEntry _yCoordinatePub;
  private DoubleEntry _xCoordinatePub;
  private DoubleArrayEntry _fieldZonesPub;

  private Timer _lockTimer = new Timer();
  private FieldZones fieldSubzones = new FieldZones();

  public TeleopSubystem(){
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Teleop");
    _columnSub = autoTable.getStringTopic("dpub/column").subscribe("-1");
    _rowSub = autoTable.getStringTopic("dpub/row").subscribe("-1");
    _shapeSub = autoTable.getStringTopic("dpub/shape").subscribe("NONE");
    _substationSub = autoTable.getStringTopic("dpub/substation").subscribe("NONE");
    _yCoordinateSub = autoTable.getDoubleTopic("dpub/selectedYCoordinate").subscribe(-1);
    _xCoordinateSub = autoTable.getDoubleTopic("dpub/selectedXCoordinate").subscribe(-1);
    _fieldZonesSub = autoTable.getDoubleArrayTopic("dpub/fieldZones").subscribe(new double[0]);
    
    _columnPub = autoTable.getStringTopic("rpub/column").getEntry("-1");
    _rowPub = autoTable.getStringTopic("rpub/row").getEntry("-1");
    _shapePub = autoTable.getStringTopic("rpub/shape").getEntry("NONE");
    _substationPub = autoTable.getStringTopic("rpub/substation").getEntry("NONE");
    _yCoordinatePub = autoTable.getDoubleTopic("rpub/selectedYCoordinate").getEntry(-1);
    _xCoordinatePub = autoTable.getDoubleTopic("rpub/selectedXCoordinate").getEntry(-1);
    _fieldZonesPub = autoTable.getDoubleArrayTopic("rpub/fieldZones")
    .getEntry(new double[0]);

    _lockTimer.start();
  }

  @Override
  public void periodic(){
    if (_lockTimer.get() > .5) {
      _columnPub.set(_columnSub.get("-1"));
      _rowPub.set(_rowSub.get("-1"));
      _shapePub.set(_shapeSub.get("NONE"));
      _yCoordinatePub.set(_yCoordinateSub.get(-1));
      _xCoordinatePub.set(_xCoordinateSub.get(-1));
      _fieldZonesPub.set(_fieldZonesSub.get(getFieldZonesAsDashboardData(fieldSubzones.getFieldZones())));
    }

    _substationPub.set(_substationSub.get("SINGLE"));
  }

  public Translation2d getTabletFieldCoordinates() {
    double x = _xCoordinateSub.get(-1);
    double y = _yCoordinateSub.get(-1);
    return new Translation2d(x, y);
  }

  /**
   * Converts a list of FieldZone objects into coordinates and dimensions sendable to the dashboard
   * @param fieldSubzone A FieldSubzone object
   * @return An array of doubles representing the subzone dimensions in the format [southwestX, southwestY, height, width]
   */
  private double[] getFieldZonesAsDashboardData(ArrayList<FieldZone> fieldZones) {
    // Create a list of field subzones from the field zones
    ArrayList<FieldSubzone> fieldSubzones = new ArrayList<FieldSubzone>();
    fieldZones.forEach((fieldZone) -> {
      fieldZone.getSubzones().forEach((fieldSubzone) -> {
        fieldSubzones.add(fieldSubzone);
      });
    });
    // Each field subzone needs to send over four pieces of information (southwest x coordinate, southwest y coordinate, height, and width)
    double[] data = new double[fieldSubzones.size() * 4];
    // Append each field subzone's data
    for (int i = 0; i < fieldSubzones.size(); i++) {
      FieldSubzone currentFieldSubzone = fieldSubzones.get(i);
      data[i*4] = currentFieldSubzone.getSouthwestCorner().getX();
      data[(i*4)+1] = currentFieldSubzone.getSouthwestCorner().getY();
      data[(i*4)+2] = currentFieldSubzone.getBoundingBox().getHeight();
      data[(i*4)+3] = currentFieldSubzone.getBoundingBox().getWidth();
    }
    // for (double item : data) {
    //   System.out.println(item);
    // }
    return data;
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

  public int GetScoringColumn() {
    try {
      return Integer.parseInt(_columnPub.get("-1"));
    } catch(NumberFormatException e) {
      return -1;
    }
  }

  public int GetScoringRow() {
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
