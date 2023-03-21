package frc.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract interface CommandSupplier {
  
  public abstract CommandBase getCommand();
}