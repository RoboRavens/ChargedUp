package frc.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoMode {
  private String _text;
  private CommandSupplier _commandSupplier;

  public AutoMode(String text) {
    this(text, null);
  }
  
  public AutoMode(String text, CommandSupplier commandSupplier) {
    _text = text;
    if (commandSupplier == null) {
      _commandSupplier = () -> { return new InstantCommand(() -> System.out.println(text)); };
    } else {
      _commandSupplier = commandSupplier;
    }
  }

  public String getText() {
    return _text;
  }

  public CommandSupplier getCommandSupplier() {
    return _commandSupplier;
  }
}