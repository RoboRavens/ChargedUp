package frc.util;

import edu.wpi.first.wpilibj2.command.Command;

public abstract interface CommandSupplier {
  
  public abstract Command getCommand();
}