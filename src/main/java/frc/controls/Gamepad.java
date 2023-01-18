package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Gamepad {
	private Joystick _joystick;

	public Gamepad(Joystick joystick) {
		_joystick = joystick;
	}

	public Gamepad(int port) {
		_joystick = new Joystick(port);
	}

	public boolean getButtonValue(ButtonCode button) {
		return _joystick.getRawButton(getButtonNumber(button));
	}

	public JoystickButton getButton(ButtonCode button) {
		return new JoystickButton(_joystick, getButtonNumber(button));
	}

	public boolean getAxisIsPressed(AxisCode axis) {
		boolean isPressed = false;
		double axisValue = this.getAxis(axis);

		if (axisValue >= Constants.AXIS_IS_PRESSED_VALUE) {
			isPressed = true;
		}

		return isPressed;
	}

	public double getAxis(AxisCode axis) {
		double axisValue;

		switch (axis) {
		case LEFTSTICKX:
			axisValue = _joystick.getRawAxis(0);
			break;
		case LEFTSTICKY:
			axisValue = _joystick.getRawAxis(1);
			break;
		case LEFTTRIGGER:
			axisValue = _joystick.getRawAxis(2);
			break;
		case RIGHTTRIGGER:
			axisValue = _joystick.getRawAxis(3);
			break;
		case RIGHTSTICKX:
			axisValue = _joystick.getRawAxis(4);
			break;
		case RIGHTSTICKY:
			axisValue = _joystick.getRawAxis(5);
			break;
		default:
			axisValue = 0;
			break;
		}
		return axisValue;
	}

	public int getButtonNumber(ButtonCode button) {
		int buttonNumber;

		switch (button) {
		case A:
			buttonNumber = 1;
			break;
		case B:
			buttonNumber = 2;
			break;
		case X:
			buttonNumber = 3;
			break;
		case Y:
			buttonNumber = 4;
			break;
		case LEFTBUMPER:
			buttonNumber = 5;
			break;
		case RIGHTBUMPER:
			buttonNumber = 6;
			break;
		case BACK:
			buttonNumber = 7;
			break;
		case START:
			buttonNumber = 8;
			break;
		case LEFTSTICK:
			buttonNumber = 9;
			break;
		case RIGHTSTICK:
			buttonNumber = 10;
			break;
		case LIMELIGHT_LIGHT_OFF_OVERRIDE:
			buttonNumber = 1;
			break;
		case SHOOTER_REV:
			buttonNumber = 2;
			break;
		case TURRET_DISABLED_OVERRIDE:
			buttonNumber = 4;
			break;
		case SHOOTER_OVERRIDE:
			buttonNumber = 7;
			break;
		case SHOOTER_LAUNCH_PAD_SHOT:
			buttonNumber = 9;
			break;
		case SHOOTER_TARMAC_SHOT:
			buttonNumber = 10;
			break;
		case SHOOTER_LOW_GOAL_SHOT:
			buttonNumber = 11;
			break;
		case SHOOTER_AUTO_RADIUS_SHOT:
			buttonNumber = 12;
			break;
		case FEEDER_WHEEL_REVERSE:
			buttonNumber = 6;
			break;
		case CLIMBER_EXTEND:
			buttonNumber = 1;
			break;
		case CLIMBER_RETRACT:
			buttonNumber = 2;
			break;
		case CLIMBER_OVERRIDE:
			buttonNumber = 3;
			break;
		case TOGGLE_COLOR_SENSING_FEATURES:
			buttonNumber = 4;
			break;
		case TURRET_HOME:
			buttonNumber = 3;
			break;
		default:
			throw new IllegalArgumentException("Invalid Button Code");
		}

		return buttonNumber;
	}

	public void setRumbleOn() {
        _joystick.setRumble(RumbleType.kLeftRumble, 1);
        _joystick.setRumble(RumbleType.kRightRumble, 1);
	}

	public void setRumbleOff() {
		_joystick.setRumble(RumbleType.kLeftRumble, 0);
        _joystick.setRumble(RumbleType.kRightRumble, 0);
	}
}