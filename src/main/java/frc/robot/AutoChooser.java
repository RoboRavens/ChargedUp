package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandSupplier;

public class AutoChooser extends SubsystemBase {
  public class AutoMode {
    public String _text;
    public CommandSupplier _commandSupplier;

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

  private SendableChooser<AutoMode> _chooser = new SendableChooser<AutoMode>();
  private String _tabName = "Autonomous";
  private ShuffleboardTab _tab = Shuffleboard.getTab(_tabName);
  private String _dashboardName = "AutoChooser";
  private GenericEntry _chosenAutoText;

  public void BuildAutoChooser(Alliance alliance) {
    switch(alliance){
      case Blue:
      this.addDefaultOption(new AutoMode("B1: Preload + mobility + balance"));
      this.addOption(new AutoMode("B2: LZ-side 2 cone + balance", () -> { return new InstantCommand(); }));
      this.addOption(new AutoMode("B3: LZ-side 2.5 cone balance"));
      this.addOption(new AutoMode("B4: LZ-side 3 cone"));
      this.addOption(new AutoMode("B5: Cable-side 2 cone balance"));
      this.addOption(new AutoMode("B6: Cable-side 2.5 balance"));
      this.addOption(new AutoMode("B7: Cable-side 3 cone"));
      break;
      case Red:
      this.addDefaultOption(new AutoMode("R1: Preload + mobility + balance"));
      this.addOption(new AutoMode("R2: LZ-side 2 cone + balance"));
      this.addOption(new AutoMode("R3: LZ-side 2.5 cone balance"));
      this.addOption(new AutoMode("R4: LZ-side 3 cone"));
      this.addOption(new AutoMode("R5: Cable-side 2 cone balance"));
      this.addOption(new AutoMode("R6: Cable-side 2.5 balance"));
      this.addOption(new AutoMode("R7: Cable-side 3 cone"));
      break;
      default:
      this.addOption(new AutoMode("Invalid Alliance"));
    }

    _tab.add(_dashboardName, _chooser)
      .withSize(4, 1);

    _chosenAutoText = _tab.add("chosen auto", "")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1)
      .withSize(4, 1)
      .getEntry();
  }

  @Override
  public void periodic() {
    var chosen = _chooser.getSelected();
    _chosenAutoText.setString(chosen.getText());
  }

  public void ShowTab() {
    Shuffleboard.selectTab(_tabName);
  }

  public CommandBase GetAuto() {
    var chosen = _chooser.getSelected();
    return chosen.getCommandSupplier().getCommand();
  }

  public void addDefaultOption(AutoMode auto) {
    _chooser.setDefaultOption(auto.getText(), auto);
  }

  private void addOption(AutoMode auto) {
    _chooser.addOption(auto.getText(), auto);
  }
}
