package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.claw.CloseClawCommand;
import frc.util.StateManagement.ClawState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;

// TODO: Implement Claw Subsystem
public class ClawSubsystem extends SubsystemBase {

    PneumaticHub pneumaticHub = new PneumaticHub(0);
    
    DoubleSolenoid rDoubleSolenoid = new DoubleSolenoid(null, 1, 3) ;
    DoubleSolenoid lDoubleSolenoid = new DoubleSolenoid(null, 5, 7) ;
    DigitalInput pieceSensor = new DigitalInput(RobotMap.PIECE_SENSOR);
 
    
    // Returns true if the sensor detects a game piece
    // And false if a game piece is not detected
    public boolean detectsGamePiece() {
        if (pieceSensor.get()) {
        return true;
        } else {
        return false;
        }
    }

    // Returns true if the claw is open
    // And false if the claw is closed
    private boolean isOpen() {   
     return Robot.clawState == ClawState.OPEN;
    }

    private boolean isClosed() {
     return Robot.clawState == ClawState.CLOSED; 
    }

    public void open() {
        Robot.clawState = ClawState.OPENING;
        lDoubleSolenoid.set(Value.kForward);
        rDoubleSolenoid.set(Value.kForward);
    }

    public void close() {
        Robot.clawState = ClawState.CLOSING;
        lDoubleSolenoid.set(Value.kReverse);
        rDoubleSolenoid.set(Value.kReverse);
    }

    public void setAndManageClawStates() {
        // Sets the claw state
        new Trigger(() -> isOpen()).whileTrue(new InstantCommand(() -> Robot.clawState = ClawState.OPEN));
        new Trigger(() -> isClosed()).whileTrue(new InstantCommand(() -> Robot.clawState = ClawState.CLOSED));
        // Sets the load state
        new Trigger(() -> detectsGamePiece() && Robot.clawState == ClawState.CLOSED).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.LOADED));
        new Trigger(() -> detectsGamePiece() && Robot.clawState != ClawState.CLOSED).whileTrue(
            new InstantCommand(() -> Robot.overallState = OverallState.LOADING)
            .andThen(new CloseClawCommand())
            .andThen(new InstantCommand(() -> {
                // Check if the game piece was loaded successfully
                if (detectsGamePiece() && isClosed()) {
                    Robot.overallState = OverallState.LOADED_TRANSIT;
                }
            })));//.schedule());
            

            /*
        }
        else {
            Robot.loadState = LoadState.EMPTY;
        }
        */
    }

    @Override
    public void periodic() {

        /* 
        setClawStates();
     //   if (Robot.clawState == ClawState.OPENING) {
     //      new robot.commands.OpenClawCommand();

            })).withName("Close claw and update overall state")
        );
        new Trigger(() -> detectsGamePiece() == false).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.EMPTY));
            */
    }
}

