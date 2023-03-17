package frc.robot.subsystems.TabletScoring;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TabletScoringSubsystem extends SubsystemBase {
  private ShuffleboardTab _tab;
  private GenericEntry _selectedRowEntry;
  private GenericEntry _selectedColumnEntry;
  private GenericEntry _coneMode;
  private GenericEntry _cubeMode;
  private GenericEntry _selectedScoringShapeEntry;
  private GenericEntry[][] _entries = new GenericEntry[3][9];

  private ScoringPosition _selectedScoringPosition = new ScoringPosition();
  private ScoringShape _selectedScoringShape = ScoringShape.NONE;

  public TabletScoringSubsystem() {
    DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");
    String text = LocalDateTime.now().format(formatter);
    String tabName = "Tablet Scoring " + text;
    _tab = Shuffleboard.getTab(tabName);
    this.InitializeWidgets();
    Shuffleboard.selectTab(tabName);
  }

  @Override
  public void periodic() {
    for (int rowIndex = 0; rowIndex < 3; rowIndex++) {
      for (int columnIndex = 0; columnIndex < 9; columnIndex++) {
        CheckButton(rowIndex, columnIndex);
      }
    }

    this.CheckConeCubeButtons();
  }

  public TabletScoringResult GetScoringTarget() {
    return new TabletScoringResult(_selectedScoringPosition, _selectedScoringShape);
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
        return "";
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

  private void InitializeWidgets() {
    _selectedRowEntry = _tab
      .add("selected row", "none")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(3, 3)
      .getEntry();

    _selectedColumnEntry = _tab
      .add("selected column", "none")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(3, 4)
      .getEntry();

    _coneMode = _tab
      .add("CONE", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(4, 3)
      .withSize(2, 2)
      .getEntry();

    _cubeMode = _tab
      .add("CUBE", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(6, 3)
      .withSize(2, 2)
      .getEntry();

    _selectedScoringShapeEntry = _tab
      .add("selected shape", "NONE")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(8, 3)
      .getEntry();

    this.AddRow("L", 2);
    this.AddRow("M", 1);
    this.AddRow("H", 0);
  }
}
