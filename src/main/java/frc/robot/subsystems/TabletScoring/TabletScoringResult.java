package frc.robot.subsystems.TabletScoring;

public class TabletScoringResult {
  private ScoringPosition _position;
  private ScoringShape _shape;

  TabletScoringResult(ScoringPosition position, ScoringShape shape) {
    _position = position;
    _shape = shape;
  }

  public ScoringPosition GetPosition() {
    return _position;
  }

  public ScoringShape GetShape() {
    return _shape;
  }
}
