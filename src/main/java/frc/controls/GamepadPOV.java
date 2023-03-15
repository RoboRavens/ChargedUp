package frc.controls;

public enum GamepadPOV {
  Up(0),
  UpRight(45),
  Right(90),
  DownRight(135),
  Down(180),
  DownLeft(225),
  Left(270),
  UpLeft(315);

  public final Integer Angle;

  private GamepadPOV(int angle){
    this.Angle = angle;
  }
}
