package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.RotateArmToRetrievalPositionCommand;
import frc.robot.commands.arm.RotateArmToRowPositionCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.arm.ExtendArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRowPositionCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.util.ArmPose;
import frc.util.StateManagementNew.ArmExtensionState;
import frc.util.StateManagementNew.LoadState;
import frc.util.StateManagementNew.LoadTargetState;
import frc.util.StateManagementNew.OverallState;
import frc.util.StateManagementNew.ZoneState;

public class ArmSubsystem extends SubsystemBase {

    public WPI_TalonFX motor1 = new WPI_TalonFX(10);
    public WPI_TalonFX motor2 = new WPI_TalonFX(11);
    public WPI_TalonSRX motorsLeader = new WPI_TalonSRX(12);

    public PIDController pidController;
    CommandXboxController _controller;

    public double _armPosition;
    public double _armVelocity;
    public double _armAcceleration;

    private ArmPose armPose = new ArmPose();
    private int armRotationFinalTarget = 0;
    private int armRotationInstantaneousTarget = 0;
    private int armExtensionFinalTarget = 0;
    private int armExtensionInstantaneousTarget = 0;

    public ArmSubsystem() {
        // configures motion magic setup
        motorsLeader.configFactoryDefault();
        motor1.configFactoryDefault();
        motor2.configFactoryDefault(100);
        motorsLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motorsLeader.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        motorsLeader.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        motorsLeader.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        motorsLeader.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        motorsLeader.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        motorsLeader.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motor1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        motor2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        // configures following
        motorsLeader.setInverted(false);
        motor2.follow(motorsLeader);
        motor2.setInverted(InvertType.FollowMaster);
        motor1.follow(motorsLeader);
        motor1.setInverted(InvertType.FollowMaster);
    }

    public void setArmPosition(double _armPosition, double _armVelocity, double _armAcceleration) {
        // always sends the motors in the right direction
        if (_armPosition - (motorsLeader.getSelectedSensorPosition()) < 0) {
            motorsLeader.configMotionCruiseVelocity(_armVelocity * -1, Constants.kTimeoutMs);
        }
        if (_armPosition - (motorsLeader.getSelectedSensorPosition()) > 0) {
            motorsLeader.configMotionCruiseVelocity(_armVelocity, Constants.kTimeoutMs);
        }
        motorsLeader.configMotionAcceleration(_armAcceleration, Constants.kTimeoutMs);
        motorsLeader.set(ControlMode.Position, _armPosition);
        SmartDashboard.putNumber("ArmSetPoint", _armPosition);
    }

    public void stopArm() {
        motorsLeader.setVoltage(0);
    }

    public void motionMagic() {
        pidController = new PIDController(Constants.kGains.kP, Constants.kGains.kI, Constants.kGains.kD);
        pidController.setSetpoint(8);
        pidController.setP(0.0);
        pidController.getPositionError();
        pidController.getPositionTolerance();
        pidController.getSetpoint();
        pidController.getP();
        pidController.getI();
        pidController.getD();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeaderEncoderPosition", motorsLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("motor1position", motor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("motor2position", motor2.getSelectedSensorPosition());
        double setPointDegrees = (360 * (SmartDashboard.getNumber("ArmSetPoint", 0.0))
                / Constants.COUNTS_PER_REVOLUTION);
        SmartDashboard.putNumber("SetPositonInDegrees", setPointDegrees);
        double armPointDegrees = (360 * (SmartDashboard.getNumber("LeaderEncoderPosition", 0.0))
                / Constants.COUNTS_PER_REVOLUTION);
        SmartDashboard.putNumber("ActualPositionInDegrees", armPointDegrees);

        armPose.calculateInstantaneousMaximums();
        setAndManageArmStates();
    }

    private void manageSetpoints() {
        int maxInstantaneousExtension = armPose.getArmLengthToHitConstraintNativeUnits();
        this.armExtensionInstantaneousTarget = Math.min(this.armExtensionFinalTarget, maxInstantaneousExtension);
    }

    private void setExtensionTarget(int encoderNativeUnits) {
        this.armExtensionFinalTarget = encoderNativeUnits;
    }

    private void setAndManageArmStates() {
        // Schedules commands that require the arm subsystem
        // The commands scheduled set the arm state in turn
        if (Robot.overallState == OverallState.EJECTING) {
            new EjectPieceCommand().schedule();
        } else if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE
                || Robot.overallState == OverallState.GROUND_PICKUP) {
            // Extend and rotate the arm to the loading target
            new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)
                    .andThen(new ExtendArmToRetrievalPositionCommand(Robot.loadTargetState)).schedule();
        } else if (Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY) {
            // Extend and rotate the arm to the scoring target
            new RotateArmToRowPositionCommand(Robot.scoringTargetState)
                    .andThen(new ExtendArmToRowPositionCommand(Robot.scoringTargetState)).schedule();
        }
        // (When the robot is either in the neutral area, any of the opposite alliance's
        // zones, or on our alliance bridge)
        else {
            // Retract the arm, but rotate the arm to the scoring/loading position
            if (Robot.loadState == LoadState.LOADED) {
                new RetractArmCommand().andThen(new RotateArmToRowPositionCommand(Robot.scoringTargetState)).schedule();
            } else {
                new RetractArmCommand().andThen(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState))
                        .schedule();
            }
        }
    }
}
