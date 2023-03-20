package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.util.StateManagement.PieceState;
import frc.util.arm.ArmPose;
import frc.util.arm.ArmSetpoint;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonFX rotationMotor1 = new WPI_TalonFX(RobotMap.ARM_ROTATION_MOTOR_1);
    private WPI_TalonFX rotationMotor2 = new WPI_TalonFX(RobotMap.ARM_ROTATION_MOTOR_2);
    private WPI_TalonSRX rotationMotorsLeader = new WPI_TalonSRX(RobotMap.ARM_ROTATION_MOTOR_LEADER);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(RobotMap.ARM_EXTENSION_MOTOR);
    private DoubleSolenoid brakeDoubleSolenoid = new DoubleSolenoid(RobotMap.REV_PNEUMATICS_MODULE_ID, PneumaticsModuleType.REVPH, RobotMap.ARM_BRAKE_DOUBLE_SOLENOID_FORWARD_CHANNEL, RobotMap.ARM_BRAKE_DOUBLE_SOLENOID_REVERSE_CHANNEL);

    private PIDController pidController;
    CommandXboxController _controller;

    private double _armRotationPosition;
    private double _armRotationVelocity;
    private double _armRotationAcceleration;

    private ArmPose armPose = new ArmPose(Constants.ARM_STARTING_DEGREES);
    private double armRotationSubSetpointFinalTargetNativeUnits = 0;
    private double armRotationFinalTargetNativeUnits = 0;
    private double armRotationInstantaneousTargetNativeUnits = 0;
    private double armExtensionSubSetpointFinalTargetNativeUnits = 0;
    private double armExtensionFinalTargetNativeUnits = 0;
    private double armExtensionInstantaneousTargetNativeUnits = 0;

    private double extensionAFF = 0;
    private double rotationAFF = 0;

    public double rotationTestPower = 0;

    public double extensionTestPower = 0;

    public ArmSubsystem() {
        // configures motion magic setup
        rotationMotorsLeader.configFactoryDefault();
        rotationMotor1.configFactoryDefault();
        rotationMotor2.configFactoryDefault(100);
        extensionMotor.configFactoryDefault();

        rotationMotorsLeader.setInverted(true);

        rotationMotor2.follow(rotationMotorsLeader);
        rotationMotor2.setInverted(InvertType.FollowMaster);
        rotationMotor1.follow(rotationMotorsLeader);
        rotationMotor1.setInverted(InvertType.FollowMaster);

        rotationMotorsLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotorsLeader.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        rotationMotorsLeader.config_kF(Constants.kSlotIdx, Constants.rotationGains.kF, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kP(Constants.kSlotIdx, Constants.rotationGains.kP, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kI(Constants.kSlotIdx, Constants.rotationGains.kI, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kD(Constants.kSlotIdx, Constants.rotationGains.kD, Constants.kTimeoutMs);
        extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        extensionMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        extensionMotor.config_kF(Constants.kSlotIdx, Constants.extensionGains.kF, Constants.kTimeoutMs);
        extensionMotor.config_kP(Constants.kSlotIdx, Constants.extensionGains.kP, Constants.kTimeoutMs);
        extensionMotor.config_kI(Constants.kSlotIdx, Constants.extensionGains.kI, Constants.kTimeoutMs);
        extensionMotor.config_kD(Constants.kSlotIdx, Constants.extensionGains.kD, Constants.kTimeoutMs);
        extensionMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotorsLeader.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotor1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotor2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        extensionMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        // Set limits. The rotation only has soft limits but the extension has a physical retraction limit switch.
        rotationMotorsLeader.configForwardSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS, 0);
        rotationMotorsLeader.configReverseSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS * -1, 0);
        rotationMotorsLeader.configForwardSoftLimitEnable(true, 0);
        rotationMotorsLeader.configReverseSoftLimitEnable(true, 0);

        extensionMotor.configForwardSoftLimitThreshold(Constants.ARM_MAX_EXTENSION_ENCODER_UNITS, 0);
        extensionMotor.configReverseSoftLimitThreshold(5000, 0);
        extensionMotor.configForwardSoftLimitEnable(true, 0);
        extensionMotor.configReverseSoftLimitEnable(true, 0);
        extensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    //set the arms relative positon encoder based off of absolute encoder
    //when absEncoder = 229 then relEncoder = 0
    public void armRotationAbsolutePosition() {

        /* 
        The absolute position of the encoder somehow changed, so I am commenting this out until further notice.

        double armRotationAbsolutePosition = rotationMotorsLeader.getSensorCollection().getPulseWidthPosition();
        

        double armRotationRelativePositionBasedOnAbsolute = (armRotationAbsolutePosition - Constants.ARM_ROTATION_ABSOLUTE_ENCODER_POSITION_AT_ZERO) * -1;

        if (armRotationAbsolutePosition < Constants.ARM_ROTATION_ABSOLUTE_ENCODER_POSITION_AT_ZERO) {
            armRotationRelativePositionBasedOnAbsolute = (armRotationAbsolutePosition * -1) + Constants.ARM_ROTATION_ABSOLUTE_ENCODER_POSITION_AT_ZERO;
        }
        
        rotationMotorsLeader.setSelectedSensorPosition(armRotationRelativePositionBasedOnAbsolute);
        
        rotationMotor1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotor2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        */
    }
  
    public void enableRotationLimit(boolean ignoreRotationLimit) {
        rotationMotorsLeader.configForwardSoftLimitEnable(ignoreRotationLimit);
        rotationMotorsLeader.configReverseSoftLimitEnable(ignoreRotationLimit);
    }

    public void enableExtensionLimit(boolean ignoreExtensionLimit) {
        extensionMotor.configForwardSoftLimitEnable(ignoreExtensionLimit);
        extensionMotor.configReverseSoftLimitEnable(ignoreExtensionLimit);
    }

    public void brakeEnable() {
        // brakeDoubleSolenoid.set(Value.kForward);
    }

    public void brakeDisable() {
        // brakeDoubleSolenoid.set(Value.kReverse);
    }

    public void setFinalTargetPositions(ArmSetpoint finalSetpoint, ArmSetpoint currentSetpoint) {
        armRotationFinalTargetNativeUnits = finalSetpoint.getRotationSetpoint();
        armExtensionFinalTargetNativeUnits = finalSetpoint.getExtensionSetpoint();
        armRotationSubSetpointFinalTargetNativeUnits = currentSetpoint.getRotationSetpoint();
        armExtensionSubSetpointFinalTargetNativeUnits = currentSetpoint.getExtensionSetpoint();
    }

    public void updateFinalTargetPositions() {
        // Length only has one constraint (maximum) so just min the constraint and the target.
        armExtensionFinalTargetNativeUnits = Math.min(armExtensionFinalTargetNativeUnits, armPose.getArmLengthToHitConstraintNativeUnits());

        // Rotation is slightly trickier since it has constraints in either direction.
        // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
        // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
        // Remember to use max instead of min when looking at bounds that are lower than the current value.
        if (_armRotationPosition < armRotationFinalTargetNativeUnits) {
            // The arm is moving forward.
            armRotationFinalTargetNativeUnits = Math.min(armRotationFinalTargetNativeUnits, armPose.getArmRotationMaximumBoundNativeUnits());
        }
        else {
            // The arm is moving backward.
            armRotationFinalTargetNativeUnits = Math.max(armRotationFinalTargetNativeUnits, armPose.getArmRotationMinimumBoundNativeUnits());
        }
    }

    /*
    public void setArmRotationAngularPosition(double _armAngle, double _armVelocity, double _armAcceleration) {
        //replace _armPosition with getPositionFromAngle(_armPosition) so that _armPosition can just be an angle
        if (getPositionFromAngle(_armAngle) - (rotationMotorsLeader.getSelectedSensorPosition()) < 0) {
          rotationMotorsLeader.configMotionCruiseVelocity(_armVelocity * -1, Constants.kTimeoutMs);
        }
        if (getPositionFromAngle(_armAngle) - (rotationMotorsLeader.getSelectedSensorPosition()) > 0) {
          rotationMotorsLeader.configMotionCruiseVelocity(_armVelocity, Constants.kTimeoutMs);
        }
    
        rotationMotorsLeader.configMotionAcceleration(_armAcceleration, Constants.kTimeoutMs);
        rotationMotorsLeader.set(ControlMode.Position, getPositionFromAngle(_armAngle));
        SmartDashboard.putNumber("ArmSetPosition", getPositionFromAngle(_armAngle));
    }
    */

    public void setArmRotationPosition(double setpoint, double rotationVelocity, double rotationAcceleration) {
        rotationMotorsLeader.configMotionCruiseVelocity(rotationVelocity, Constants.kTimeoutMs);
        rotationMotorsLeader.configMotionAcceleration(rotationAcceleration, Constants.kTimeoutMs);
        rotationMotorsLeader.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);
        /*
        SmartDashboard.putNumber("ArmRotationPosition", setpoint);
        SmartDashboard.putNumber("Error", rotationMotorsLeader.getClosedLoopError());
        SmartDashboard.putNumber("CL Target", rotationMotorsLeader.getClosedLoopTarget());
        SmartDashboard.putNumber("ATP", rotationMotorsLeader.getActiveTrajectoryPosition());
        */
    }

    public void setArmExtensionPosition(double setpoint, double extensionVelocity, double extensionAcceleration) {
        extensionMotor.configMotionCruiseVelocity(extensionVelocity, Constants.kTimeoutMs);
        extensionMotor.configMotionAcceleration(extensionAcceleration, Constants.kTimeoutMs);
        extensionMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, extensionAFF);
        // SmartDashboard.putNumber("ArmExtensionPosition", setpoint);
    }

    public void stopRotation() {
        brakeEnable();
        rotationMotorsLeader.stopMotor();
    }

    public void stopExtension() {
        extensionMotor.stopMotor();
    }

    public void armFullStop() {
        stopRotation();
        stopExtension();

        // This code will apply a constant voltage to the arm in order to fight backlash,
        // but we probably don't want this running until the brake is hooked up and working.
        // We may further want it on some kind of override later on,
        // or perhaps an override that switches to a different default arm command
        // that will seek a current position (if the brake is not online.)
        /*
        // Apply voltage in the direction that the arm is planning on going next.
        if (armRotationFinalTarget > getCurrentRotationNativeUnits()) {
            rotationMotorsLeader.setVoltage(Constants.ARM_BRAKE_ANTI_BACKLASH_VOLTAGE);
        }
        else {
            rotationMotorsLeader.setVoltage(Constants.ARM_BRAKE_ANTI_BACKLASH_VOLTAGE * -1);
        }
        */
    }

    public void motionMagic() {
        pidController = new PIDController(Constants.rotationGains.kP, Constants.rotationGains.kI, Constants.rotationGains.kD);
        pidController.setSetpoint(8);
        pidController.setP(0.0);
        pidController.getPositionError();
        pidController.getPositionTolerance();
        pidController.getSetpoint();
        pidController.getP();
        pidController.getI();
        pidController.getD();
    }

    public double getCommandTimeoutSeconds() {
        double rotationDifference = Math.abs(rotationMotorsLeader.getSelectedSensorPosition() - armRotationFinalTargetNativeUnits);
        double extensionDifference = Math.abs(extensionMotor.getSelectedSensorPosition() - armExtensionFinalTargetNativeUnits);

        double rotationTime = rotationDifference / Constants.ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
        double extensionTime = extensionDifference / Constants.ARM_EXTENSION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
        
        return Math.max(rotationTime, extensionTime) + Constants.ARM_TIMEOUT_BASE_VALUE;
        
    }

    @Override
    public void periodic() {
        armPose.setArmAngleDegrees(getCurrentAngleDegrees());

        
        SmartDashboard.putNumber("Ext Encder", extensionMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber("LeaderEncoderPosition", rotationMotorsLeader.getSelectedSensorPosition());
        
        SmartDashboard.putNumber("Absolute position", rotationMotorsLeader.getSensorCollection().getPulseWidthPosition());
        //SmartDashboard.putNumber("motor1position", rotationMotor1.getSelectedSensorPosition());
        //SmartDashboard.putNumber("motor2position", rotationMotor2.getSelectedSensorPosition());

        // double setAngle = getAngleFromPosition((SmartDashboard.getNumber("ArmSetPosition", 0.0)));
        // double setAngle = getAngleFromPosition((SmartDashboard.getNumber("ArmSetPosition", 0.0)));
        // arm.get
        // double setAngle = this.
        // SmartDashboard.putNumber("SetPositonInDegrees", setAngle);


        // double actualAngle = getAngleFromPosition((SmartDashboard.getNumber("LeaderEncoderPosition", 0.0)));
        
        SmartDashboard.putNumber("ActualPositionInDegrees", getCurrentAngleDegrees());
        
        // Angle (in 360) = 360 * ((ArmSetPosition / CountsPerRevolution) % 360)
        // ArmSetPosition = (Angle/360) * CountsPerRevolution
        // from degrees to position is ~11.377778

        armPose.calculateInstantaneousMaximums();
        this.updateInstantaneousMaximums();
        // this.updateFinalTargetPositions();
        this.updateAFFs();
    }

    private void updateInstantaneousMaximums() {
        armExtensionInstantaneousTargetNativeUnits = Math.min(armPose.getArmLengthToHitConstraintNativeUnits(), armExtensionFinalTargetNativeUnits);

        // Update based on which direction the arm is moving.
        if (armPose.getArmRotationNativeUnits() < this.armRotationFinalTargetNativeUnits) {
            armRotationInstantaneousTargetNativeUnits = Math.min(armPose.getArmRotationMaximumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
        }
        else {
            armRotationInstantaneousTargetNativeUnits = Math.max(armPose.getArmRotationMinimumBoundNativeUnits(), armRotationFinalTargetNativeUnits);
        }
/*
        SmartDashboard.putBoolean("UIM IF", (armPose.getArmRotationNativeUnits() < this.armRotationFinalTargetNativeUnits));
        
        SmartDashboard.putNumber("ARM ROT MIN", Math.min(armPose.getArmRotationMaximumBoundNativeUnits(), armRotationFinalTargetNativeUnits));
        
        SmartDashboard.putNumber("ARM ROT MAX", Math.max(armPose.getArmRotationMinimumBoundNativeUnits(), armRotationFinalTargetNativeUnits));
        

        SmartDashboard.putNumber("ExtInstTarg", armExtensionInstantaneousTargetNativeUnits);
        
        SmartDashboard.putNumber("RotInstTarg", armRotationInstantaneousTargetNativeUnits);
        SmartDashboard.putNumber("ExtFinalTarg", armExtensionFinalTargetNativeUnits);
        
        SmartDashboard.putNumber("RotFinalTarg", armRotationFinalTargetNativeUnits);
    */
    }

    public void updateAFFs() {
        updateRotationAFF();
        updateExtensionAFF();

    }

    public void updateRotationAFF() {
        double extensionPercent = getExtensionPercentOfMaximumNativeUnits();
        double armExtensionPercentTerm = (1 - Constants.ARM_MINIMUM_EXTENSION_RATIO) * extensionPercent;
        double extensionScaling = Constants.ARM_MINIMUM_EXTENSION_RATIO + armExtensionPercentTerm;

        double rotationScaling = Math.sin(Math.abs(armPose.getArmAngleRadians()));

        double maxAFF = Constants.ROTATION_SIDEWAYS_EMPTY_AFF;

        if (Robot.hasCone()) {
            maxAFF = Constants.ROTATION_SIDEWAYS_LOADED_AFF;
        }

        if (this.getCurrentAngleDegrees() > 0) {
            maxAFF = maxAFF * -1;
        }

        rotationAFF = maxAFF * rotationScaling * extensionScaling;
    }

    public void updateExtensionAFF() {
        double maxAFF = Constants.EXTENSION_UPRIGHT_EMPTY_AFF;

        if (Robot.hasCone()) {
            maxAFF = Constants.EXTENSION_UPRIGHT_LOADED_AFF;
        }
        
        double rotationScaling = Math.sin(Math.abs(armPose.getArmAngleRadians()));

        extensionAFF = maxAFF * rotationScaling;
    }

    public double getExtensionPercentOfMaximumNativeUnits() {
        return this.getCurrentExtensionNativeUnits() / Constants.ARM_MAX_EXTENSION_ENCODER_UNITS;
    }

    public double getCurrentAngleDegrees() {
        return rotationMotorsLeader.getSelectedSensorPosition() / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
    }

    public double getCurrentRotationNativeUnits() {
        return rotationMotorsLeader.getSelectedSensorPosition();
    }

    public double getCurrentExtensionNativeUnits() {
        return extensionMotor.getSelectedSensorPosition();
    }

    /*
    public double getAngleFromPosition(double position) {
        double angle =  (360 * (((position) / Constants.COUNTS_PER_REVOLUTION) % 360));
        if (angle < -180) {
            return angle + 360;
        }
        if (angle > 180) {
            return angle - 360;
        } else {
            return angle;
        }
    }
    */

    public double getPositionFromAngle(double intendedAngle) {
        double position = (intendedAngle/360) * Constants.COUNTS_PER_REVOLUTION;
          return position;
      }

    private void manageSetpoints() {
        int maxInstantaneousExtension = armPose.getArmLengthToHitConstraintNativeUnits();
        this.armExtensionInstantaneousTargetNativeUnits = Math.min(this.armExtensionFinalTargetNativeUnits, maxInstantaneousExtension);
    }

    private void setExtensionTarget(int encoderNativeUnits) {
        this.armExtensionFinalTargetNativeUnits = encoderNativeUnits;
    }

    public double getArmExtensionFinalTargetNativeUnits() {
        return this.armExtensionFinalTargetNativeUnits;
    }

    private void setRotationTarget(int encoderNativeUnits) {
        this.armRotationFinalTargetNativeUnits = encoderNativeUnits;
    }

    public double getArmRotationFinalTargetNativeUnits() {
        return this.armRotationFinalTargetNativeUnits;
    }

    public double getArmRotationInstantaneousTargetNativeUnits() {
        return armRotationInstantaneousTargetNativeUnits;
    }

    public double getArmExtensionInstantaneousTargetNativeUnits() {
        return armExtensionInstantaneousTargetNativeUnits;
    }

    public void runRotationAtTestPower() {
        rotationMotorsLeader.setVoltage(rotationTestPower);
    }

    public void runExtensionAtTestPower() {
        extensionMotor.set(extensionTestPower);
    }

    public void moveArmToTarget() {
        ArmSetpoint targetArmSetpoint;
        switch (Robot.scoringTargetState) {
            case HIGH:
              if (Robot.pieceState == PieceState.CONE) {
                targetArmSetpoint = Constants.ARM_SCORE_CONE_HIGH_SETPOINT;
              }
              else {
                targetArmSetpoint = Constants.ARM_SCORE_CUBE_HIGH_SETPOINT;
              }
              break;
            case MID:
              if (Robot.pieceState == PieceState.CONE) {
                targetArmSetpoint = Constants.ARM_SCORE_CONE_MID_SETPOINT;
              }
              else {
                targetArmSetpoint = Constants.ARM_SCORE_CUBE_MID_SETPOINT;
              }
              break;
            case LOW:
              targetArmSetpoint = Constants.ARM_SCORE_LOW_SETPOINT;
              break;
            default:
              targetArmSetpoint = Constants.ARM_FULL_RETRACT_SETPOINT;
              break;
          }
          
          new ArmGoToSetpointDangerousCommand(targetArmSetpoint).schedule();
    }

    public void setRotationVoltage(double voltage) {
        rotationMotorsLeader.setVoltage(voltage);
    }

    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }
}
