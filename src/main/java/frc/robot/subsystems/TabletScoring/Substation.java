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
}
