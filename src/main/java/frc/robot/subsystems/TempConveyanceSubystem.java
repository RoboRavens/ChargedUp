package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class TempConveyanceSubystem extends SubsystemBase {
    private WPI_TalonSRX _conveyanceMotorOne;

    public TempConveyanceSubystem() {
        _conveyanceMotorOne = new WPI_TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    }

    private void runConveyanceAtVoltage(double magnitude) {
        // this._conveyanceMotorOne.set(ControlMode.PercentOutput, magnitude);
        
        // The better way to do this would be to update the constant values on
        // the methods that call this, but until we know it works I don't want
        // to change all the code to do that so we'll just do the conversion here.
        double voltage = magnitude * 12.0;
        this._conveyanceMotorOne.setVoltage(voltage);
    }

    public void setConveyanceIntakeCargo() {
        this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_FULL_SPEED);
    }

    public void stopConveyanceOne() {
        this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_STOP);
    }
}
