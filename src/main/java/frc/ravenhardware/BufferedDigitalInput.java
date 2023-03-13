package frc.ravenhardware;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.DigitalInput;

public class BufferedDigitalInput {
	private DigitalInput _digitalInput;
	private boolean _valueWhenTrue;
	private int _listSize;
	private LinkedList<Boolean> _sensorValues;

	public BufferedDigitalInput(int channel, int bufferSize, boolean valueWhenTrue, boolean defaultSensorValue) {
		_digitalInput = new DigitalInput(channel);
		_sensorValues = new LinkedList<Boolean>();
		_listSize = bufferSize;
		_valueWhenTrue = valueWhenTrue;
		for (int i = 0; i < _listSize; i++) {
			_sensorValues.add(defaultSensorValue == _valueWhenTrue);
		}
	}

	// Adds the current sensor value to the list, and
	// removes the first item if the list is larger than the list size.
	public void maintainState() {
		_sensorValues.add(_digitalInput.get());
		if (_sensorValues.size() > _listSize) {
			_sensorValues.remove();
		}
	}

	public boolean get() {
		int trues = 0;

		// Count the instances of "true" in the list values.
		for (Boolean value : _sensorValues) {
			if (value) {
				trues++;
			}
		}

		// If trues is greater than half the list size, return true. Otherwise, false.
		return (trues * 2 > _listSize) == _valueWhenTrue;
	}
}