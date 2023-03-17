package frc.robot.subsystems.TabletScoring;

public class ScoringPosition {
  public static final ScoringPosition NONE = new ScoringPosition();
  private Integer Row = null;
  private Integer Column = null;

  public ScoringPosition() {

  }

  public ScoringPosition(int row, int column) {
    this.Row = row;
    this.Column = column;
  }

  public int GetRow() {
    if (this.Row == null) {
      return -1;
    }
    else {
      return this.Row;
    }
  }

  public int GetColumn() {
    if (this.Column == null) {
      return -1;
    }
    else {
      return this.Column;
    }
  }

  public boolean IsNoneSelected() {
    return this.Row == null && this.Column == null;
  }

  public boolean Equal(int row, int column) {
    if (this.Row == null || this.Column == null) {
      return false;
    }

    return this.Row == row && this.Column == column;
  }

  public boolean RowEquals(int row) {
    if (this.Row == null) {
      return false;
    }

    return this.Row == row;
  }

  public boolean ColumnEquals(int column) {
    if (this.Column == null) {
      return false;
    }

    return this.Column == column;
  }
}