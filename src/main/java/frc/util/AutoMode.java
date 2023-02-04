package frc.util;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoMode {
    private String _name;
    private Command _command;

    public AutoMode(String name, Command command) {
        _name = name;
        _command = command;
    }

    public String getAutoName() {
        return _name;
    }

    public Command getAutoCommand() {
        return _command;
    }
}