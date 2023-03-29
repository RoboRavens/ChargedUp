package frc.robot.subsystems;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.auto.*;
import frc.util.AutoMode;

public class AutoChooserSubsystemReact extends SubsystemBase {
  private final Map<String, AutoMode> _autos = new LinkedHashMap<>();

  private AutoMode _redDefault;
  private AutoMode _blueDefault;
  private Alliance _currentAlliance = Alliance.Invalid;

  private StringArrayPublisher _optionsPub;
  private StringSubscriber _selectedAutoSub;
  private StringPublisher _selectedAutoRobotPub;

  private DoublePublisher _matchTimePub;
  private StringPublisher _alliancePub;

  public AutoChooserSubsystemReact() {
    var autoTable = ReactBoardSubsystem.ReactDash.getSubTable("Autonomous");
    _optionsPub = autoTable.getStringArrayTopic("rpub/options").publish();
    _selectedAutoSub = autoTable.getStringTopic("dpub/selectedAuto").subscribe(null);
    _selectedAutoRobotPub = autoTable.getStringTopic("rpub/selectedAuto").publish();

    _matchTimePub = autoTable.getDoubleTopic("rpub/matchTime").publish();
    _alliancePub = autoTable.getStringTopic("rpub/alliance").publish();
    
    // BLUE SIDE
    this.addOption(
      new AutoMode("B1: Preload + mobility + balance",
      () -> PreloadMobilityAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue))
    );
    this.addOption(
      new AutoMode("B2: LZ-side 2 cone + balance",
      () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("B3: LZ-side 2.5 cone balance", 
      () -> TwoPointFiveConeAndBalanceCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("B4: LZ-side 3 cone", 
      () -> ThreeConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("B5: Cable-side 2 cone balance",
      () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.Cable))
    );
    this.addOption(
      new AutoMode("B6: Cable-side 2.5 balance",
      () -> TwoPointFiveConeAndBalanceCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.Cable))
    );
    this.addOption(
      new AutoMode("B7: Cable-side 3 cone",
      () -> ThreeConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.Cable))
    );
    this.addBlueDefault(
      new AutoMode("B8: Mobility",
      () -> MobilityAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue))
    );
    this.addOption(
      new AutoMode("B9: Score and Mobility",
      () -> MobilityAndScoreConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue))
    );
    this.addOption(
      new AutoMode("B10: Score 2 Cones",
      () -> TwoConeAndMobilityAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("B11: Preload + Balance",
      () -> PreloadBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Blue))
    );
    this.addOption(
      new AutoMode("B12: Balance Auto V3",
      () -> BalanceAutoV3Command.getAutoMode(AutoEnums.AutoAlliance.Blue))
    );

    // RED SIDE
    this.addOption(
      new AutoMode("R1: Preload + mobility + balance",
      () -> PreloadMobilityAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red))
    );
    this.addOption(
      new AutoMode("R2: LZ-side 2 cone + balance",
      () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("R3: LZ-side 2.5 cone balance", 
      () -> TwoPointFiveConeAndBalanceCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("R4: LZ-side 3 cone", 
      () -> ThreeConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("R5: Cable-side 2 cone balance",
      () -> TwoConeAndBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.Cable))
    );
    this.addOption(
      new AutoMode("R6: Cable-side 2.5 balance",
      () -> TwoPointFiveConeAndBalanceCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.Cable))
    );
    this.addOption(
      new AutoMode("R7: Cable-side 3 cone",
      () -> ThreeConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.Cable))
    );
    this.addRedDefault(
      new AutoMode("R8: Mobility",
      () -> MobilityAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red))
    );
    this.addOption(
      new AutoMode("R9: Score and Mobility",
      () -> MobilityAndScoreConeAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red))
    );
    this.addOption(
      new AutoMode("R10: Score 2 Cones",
      () -> TwoConeAndMobilityAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red, AutoEnums.AutoSide.LZ))
    );
    this.addOption(
      new AutoMode("R11: Preload and Balance",
      () -> PreloadBalanceAutoCommand.getAutoMode(AutoEnums.AutoAlliance.Red))
    );
    this.addOption(
      new AutoMode("R12: Balance Auto V3",
      () -> BalanceAutoV3Command.getAutoMode(AutoEnums.AutoAlliance.Red))
    );
  }

  private void addOption(AutoMode auto) {
    _autos.put(auto.getText(), auto);
  }

  private void addBlueDefault(AutoMode auto) {
    _blueDefault = auto;
    _autos.put(auto.getText(), auto);
  }

  private void addRedDefault(AutoMode auto) {
    _redDefault = auto;
    _autos.put(auto.getText(), auto);
  }

  private AutoMode GetAuto() {
    String selectedAuto = _selectedAutoSub.get(null);
    if (selectedAuto == null) {
      return this.GetDefaultAuto();
    }

    var chosen = _autos.getOrDefault(selectedAuto, null);
    if (chosen == null) {
      return this.GetDefaultAuto();
    }

    return chosen;
  }

  public Command GetAutoCommand() {
    return this.GetAuto().getCommandSupplier().getCommand();
  }

  private AutoMode GetDefaultAuto() {
    return DriverStation.getAlliance() == Alliance.Blue ? _blueDefault: _redDefault;
  }

  private Command GetDefaultAutoCommand() {
    var cmdSupplier = this.GetDefaultAuto().getCommandSupplier();

    return cmdSupplier.getCommand();
  }

  private void UpdateAlliance(Alliance alliance) {
    var startsWith = alliance == Alliance.Blue ? "B" : "R";
    var keys = _autos.keySet().stream().filter(s -> s.startsWith(startsWith)).toArray(String[]::new);
    _optionsPub.set(keys);
    _currentAlliance = alliance;
  }

  @Override
  public void periodic() {
    var alliance = DriverStation.getAlliance();
    if (_currentAlliance != alliance) {
      this.UpdateAlliance(alliance);
    }

    _selectedAutoRobotPub.set(this.GetAuto().getText());

    _matchTimePub.set(Timer.getMatchTime());
    _alliancePub.set(DriverStation.getAlliance().name());
  }
}
