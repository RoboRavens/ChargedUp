package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.StateManagement.ClawState;
import frc.controls.ButtonCode;
import frc.ravenhardware.BufferedDigitalInput;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid leftDoubleSolenoid = new DoubleSolenoid(RobotMap.REV_PNEUMATICS_MODULE_ID, PneumaticsModuleType.REVPH, RobotMap.CLAW_LEFT_DOUBLE_SOLENOID_FORWARD_CHANNEL, RobotMap.CLAW_LEFT_DOUBLE_SOLENOID_REVERSE_CHANNEL) ;
    DoubleSolenoid rightDoubleSolenoid = new DoubleSolenoid(RobotMap.REV_PNEUMATICS_MODULE_ID, PneumaticsModuleType.REVPH, RobotMap.CLAW_RIGHT_DOUBLE_SOLENOID_FORWARD_CHANNEL, RobotMap.CLAW_RIGHT_DOUBLE_SOLENOID_REVERSE_CHANNEL) ;
    BufferedDigitalInput pieceSensor = new BufferedDigitalInput(RobotMap.PIECE_SENSOR, 9, false, false);
    Timer closeLockoutTimer = new Timer();

    public ClawSubsystem() {
        startTimer();
    }

    // Returns true if the sensor detects a game piece,
    // and false if a game piece is not detected.
    
    public boolean detectsGamePiece() {
        if (Robot.PIECE_SENSOR_OVERRIDE) {
            return Robot.clawState == ClawState.CLOSED || Robot.clawState == ClawState.CLOSING;
        }

        if (pieceSensor.get()) {
            return true;
        }
        else {
            return false;
        }
    }

    public void startTimer() {
        closeLockoutTimer.reset();
        closeLockoutTimer.start();
    }

    public boolean lockoutHasElapsed() {
        // System.out.println();
        return closeLockoutTimer.hasElapsed(Constants.CLAW_LOCKOUT_DURATION_SECONDS);
    }

    // Returns true if the claw is open,
    // and false if the claw is is in any other state.
    public boolean isOpen() {   
        return Robot.clawState == ClawState.OPEN;
    }

    public boolean isClosed() {
        return Robot.clawState == ClawState.CLOSED; 
    }

    public void open() {
        setClawState(ClawState.OPENING);
        leftDoubleSolenoid.set(Value.kReverse);
        rightDoubleSolenoid.set(Value.kReverse);
    }

    public void close() {
        leftDoubleSolenoid.set(Value.kForward);
        rightDoubleSolenoid.set(Value.kForward);
    }

    public void setClawState(ClawState newState) {
        Robot.clawState = newState;
    }

    @Override
    public void periodic() {
        pieceSensor.maintainState();
        SmartDashboard.putBoolean("Sense game piece", detectsGamePiece());
        SmartDashboard.putString("Clawstate", Robot.clawState.name());
    }
}

