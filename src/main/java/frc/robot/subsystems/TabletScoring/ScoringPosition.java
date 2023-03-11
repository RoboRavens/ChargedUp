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
    return this.Row;
  }

  public int GetColumn() {
    return this.Column;
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
}