package frc.robot.subsystems.TabletScoring;

public enum Substation {
  NONE(-1),
  DOUBLE_LEFT(0),
  DOUBLE_RIGHT(1),
  SINGLE(2);

  public int Index;
  
  private Substation(int index) {
    this.Index = index;
  }

  public boolean isDoubleSubstation() {
    return this == Substation.DOUBLE_LEFT || this == Substation.DOUBLE_RIGHT;
  }

  public boolean isSingleSubstation() {
    return this == Substation.SINGLE;
  }
}
