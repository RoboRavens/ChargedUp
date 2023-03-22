package frc.robot.subsystems.TabletScoring;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.field.FieldMeasurements;

public class TabletScoringSubsystem extends SubsystemBase {
  private ShuffleboardTab _tab;
  private GenericEntry _selectedRowEntry;
  private GenericEntry _selectedColumnEntry;
  private GenericEntry _coneMode;
  private GenericEntry _cubeMode;
  private GenericEntry _selectedScoringShapeEntry;
  private GenericEntry _gameTimeEntry;
  private GenericEntry[][] _entries = new GenericEntry[3][9];

  private ScoringPosition _selectedScoringPosition = new ScoringPosition();
  private ScoringShape _selectedScoringShape = ScoringShape.NONE;

  private String _tabName = "Tablet Scoring";

  private Substation _selectedSubstation = Substation.NONE;
  private GenericEntry _selectedSubstationEntry;
  private GenericEntry _substationDblLeft;
  private GenericEntry _substationDblRight;
  private GenericEntry _substationSingle;

  private GenericEntry _autoTabGo;

  public TabletScoringSubsystem() {
    //DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
    //String text = LocalDateTime.now().format(formatter);
    //String tabName = "Tablet Scoring " + text;
    _tab = Shuffleboard.getTab(_tabName);
    this.InitializeWidgets();
  }

  public void ShowTab() {
    Shuffleboard.selectTab(_tabName);
  }

  @Override
  public void periodic() {
    for (int rowIndex = 0; rowIndex < 3; rowIndex++) {
      for (int columnIndex = 0; columnIndex < 9; columnIndex++) {
        CheckButton(rowIndex, columnIndex);
      }
    }

    this.CheckConeCubeButtons();
    this.CheckSubstationButtons();
    _gameTimeEntry.setDouble(Timer.getMatchTime());

    if(_autoTabGo.getBoolean(false)) {
      Robot.AUTO_CHOOSER.ShowTab();
      _autoTabGo.setBoolean(false);
    }
  }
  
  public boolean isCubeColumn() {
    if (this.GetScoringPosition().ColumnEquals(1) ||
      this.GetScoringPosition().ColumnEquals(4) ||
      this.GetScoringPosition().ColumnEquals(7)) {
      return true;
    } 
    
    return false;
  }

  public ScoringPosition GetScoringPosition() {
    return _selectedScoringPosition;
  }

  public Translation2d GetScoringCoordinates() {
    if (_selectedScoringPosition.IsNoneSelected()) {
      return null;
    }

    if (Robot.allianceColor == Alliance.Invalid) {
      return null;
    }

    int column = _selectedScoringPosition.GetColumn();
    return Robot.allianceColor == Alliance.Blue ?
      FieldMeasurements.BLUE_NODE_COORD[column] : FieldMeasurements.RED_NODE_COORD[column];
  }

  public ScoringShape GetScoringShape() {
    return _selectedScoringShape;
  }

  public Substation GetSubstation() {
    return _selectedSubstation;
  }

  private void CheckConeCubeButtons() {
    boolean cone = _coneMode.getBoolean(false);
    boolean cube = _cubeMode.getBoolean(false);
    var sss = _selectedScoringShape;
    if (cone && sss != ScoringShape.CONE) {
      this.SetSelectedScoringShape(ScoringShape.CONE);
    } else if (cube && sss != ScoringShape.CUBE) {
      this.SetSelectedScoringShape(ScoringShape.CUBE);
    } else if (cone == false && cube == false && sss != ScoringShape.NONE) {
      this.SetSelectedScoringShape(ScoringShape.NONE);
    }
  }

  private void CheckButton(int rowIndex, int columnIndex) {
    boolean val = _entries[rowIndex][columnIndex].getBoolean(false);
    var ssp = _selectedScoringPosition;
    if (val && ssp.Equal(rowIndex, columnIndex) == false) {
      if (ssp.IsNoneSelected() == false) {
        _entries[ssp.GetRow()][ssp.GetColumn()].setBoolean(false);
      }

      this.SetSelectedScoringPosition(new ScoringPosition(rowIndex, columnIndex));
    } else if (val == false && ssp.Equal(rowIndex, columnIndex)) {
      this.SetSelectedScoringPosition(ScoringPosition.NONE);
    }
  }

  private void SetSelectedScoringPosition(ScoringPosition sp){
    _selectedScoringPosition = sp;
    if (sp == ScoringPosition.NONE) {
      _selectedRowEntry.setString("none");
      _selectedColumnEntry.setString("none");
    } else {
      _selectedRowEntry.setString(Integer.toString(sp.GetRow()));
      _selectedColumnEntry.setString(Integer.toString(sp.GetColumn()));
    }
  }

  private void AddRow(String prefix, int rowIndex) {
    for (int i = 0; i < 9; i++) {
      _entries[rowIndex][i] = _tab
          .add(prefix + i + GetPieceVisual(i), false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(i, rowIndex)
          .getEntry();
    }
  }

  private String GetPieceVisual(int columnIndex) {
    switch (columnIndex) {
      case 1:
      case 4:
      case 7:
        return "[]";
      default:
        return "";
    }
  }

  private void SetSelectedScoringShape(ScoringShape ss){
    _selectedScoringShape = ss;
    switch(ss) {
      case NONE:
        _selectedScoringShapeEntry.setString("NONE");
        _coneMode.setBoolean(false);
        _cubeMode.setBoolean(false);
        break;
      case CONE:
        _selectedScoringShapeEntry.setString("CONE");
        _coneMode.setBoolean(true);
        _cubeMode.setBoolean(false);
        break;
      case CUBE:
        _selectedScoringShapeEntry.setString("CUBE");
        _coneMode.setBoolean(false);
        _cubeMode.setBoolean(true);
        break;
    }
  }

  private void SetupSubstations() {
    _substationDblLeft = _tab
      .add("DBL LEFT", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(7, 3)
      .withSize(1, 1)
      .getEntry();

    _substationDblRight = _tab
      .add("DBL RIGHT", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(8, 3)
      .withSize(1, 1)
      .getEntry();

    _substationSingle = _tab
      .add("SINGLE SUB", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(6, 4)
      .withSize(1, 1)
      .getEntry();
  }

  private void CheckSubstationButtons() {
    boolean left = _substationDblLeft.getBoolean(false);
    boolean right = _substationDblRight.getBoolean(false);
    boolean single = _substationSingle.getBoolean(false);
    var ss = _selectedSubstation;
    if (left && ss != Substation.DOUBLE_LEFT) {
      this.SetSelectedSubstation(Substation.DOUBLE_LEFT);
    } else if (right && ss != Substation.DOUBLE_RIGHT) {
      this.SetSelectedSubstation(Substation.DOUBLE_RIGHT);
    } else if (single && ss != Substation.SINGLE) {
      this.SetSelectedSubstation(Substation.SINGLE);
    } else if (left == false && right == false && single == false && ss != Substation.NONE) {
      this.SetSelectedSubstation(Substation.NONE);
    }
  }

  private void SetSelectedSubstation(Substation s){
    _selectedSubstation = s;
    _selectedSubstationEntry.setString(s.name());
    switch(s) {
      case NONE:
        _substationDblLeft.setBoolean(false);
        _substationDblRight.setBoolean(false);
        _substationSingle.setBoolean(false);
        break;
      case DOUBLE_LEFT:
        _substationDblLeft.setBoolean(true);
        _substationDblRight.setBoolean(false);
        _substationSingle.setBoolean(false);
        break;
      case DOUBLE_RIGHT:
        _substationDblLeft.setBoolean(false);
        _substationDblRight.setBoolean(true);
        _substationSingle.setBoolean(false);
        break;
      case SINGLE:
        _substationDblLeft.setBoolean(false);
        _substationDblRight.setBoolean(false);
        _substationSingle.setBoolean(true);
        break;
    }
  }

  private void InitializeWidgets() {
    _selectedRowEntry = _tab
      .add("selected row", "none")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 3)
      .getEntry();

    _selectedColumnEntry = _tab
      .add("selected column", "none")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 4)
      .getEntry();

    _coneMode = _tab
      .add("CONE", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(2, 3)
      .withSize(2, 2)
      .getEntry();

    _cubeMode = _tab
      .add("CUBE", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(4, 3)
      .withSize(2, 2)
      .getEntry();

    _selectedScoringShapeEntry = _tab
      .add("selected shape", "NONE")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 3)
      .getEntry();

    _selectedSubstationEntry = _tab
      .add("selected substation", "NONE")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(1, 4)
      .getEntry();

    _gameTimeEntry = _tab
      .add("Game Time", 0.0)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 135))
      .withPosition(9, 0)
      .withSize(2, 2)
      .getEntry();

    _autoTabGo = _tab
      .add("auto tab", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(10, 2)
      .withSize(1, 1)
      .getEntry();

    this.AddRow("L", 2);
    this.AddRow("M", 1);
    this.AddRow("H", 0);
    this.SetupSubstations();
  }
}
