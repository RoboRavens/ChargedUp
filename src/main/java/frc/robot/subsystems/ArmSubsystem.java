package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.arm.ArmPose;
import frc.util.arm.ArmSetpoint;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonFX rotationMotor1 = new WPI_TalonFX(RobotMap.ARM_ROTATION_MOTOR_1);
    private WPI_TalonFX rotationMotor2 = new WPI_TalonFX(RobotMap.ARM_ROTATION_MOTOR_2);
    private WPI_TalonSRX rotationMotorsLeader = new WPI_TalonSRX(RobotMap.ARM_ROTATION_MOTOR_LEADER);
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(RobotMap.ARM_EXTENSION_MOTOR);
    private DoubleSolenoid brakeDoubleSolenoid = new DoubleSolenoid(null, RobotMap.ARM_BRAKE_DOUBLE_SOLENOID_FORWARD_CHANNEL, RobotMap.ARM_BRAKE_DOUBLE_SOLENOID_REVERSE_CHANNEL) ;

    private PIDController pidController;
    CommandXboxController _controller;

    private double _armRotationPosition;
    private double _armRotationVelocity;
    private double _armRotationAcceleration;

    private ArmPose armPose = new ArmPose(Constants.ARM_STARTING_DEGREES);
    private double armRotationFinalTarget = 0;
    private double armRotationInstantaneousTarget = 0;
    private double armExtensionFinalTarget = 0;
    private double armExtensionInstantaneousTarget = 0;

    private double extensionAFF = 0;
    private double rotationAFF = 0;

    public ArmSubsystem() {
        // configures motion magic setup
        rotationMotorsLeader.configFactoryDefault();
        rotationMotor1.configFactoryDefault();
        rotationMotor2.configFactoryDefault(100);
        rotationMotorsLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotorsLeader.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        rotationMotorsLeader.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        rotationMotorsLeader.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        rotationMotorsLeader.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotor1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rotationMotor2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        // configures following
        rotationMotorsLeader.setInverted(false);
        rotationMotor2.follow(rotationMotorsLeader);
        rotationMotor2.setInverted(InvertType.FollowMaster);
        rotationMotor1.follow(rotationMotorsLeader);
        rotationMotor1.setInverted(InvertType.FollowMaster);

        // Set limits. The rotation only has soft limits but the extension has a physical retraction limit switch.
        rotationMotorsLeader.configForwardSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS, 0);
        rotationMotorsLeader.configReverseSoftLimitThreshold(Constants.ARM_ROTATION_MAXIMUM_ENCODER_UNITS * -1, 0);
        rotationMotorsLeader.configForwardSoftLimitEnable(true, 0);
        rotationMotorsLeader.configReverseSoftLimitEnable(true, 0);

        extensionMotor.configForwardSoftLimitThreshold(Constants.ARM_MAX_EXTENSION_ENCODER_UNITS, 0);
        extensionMotor.configReverseSoftLimitThreshold(0, 0);
        extensionMotor.configForwardSoftLimitEnable(true, 0);
        extensionMotor.configReverseSoftLimitEnable(true, 0);
        extensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    public void brakeEnable() {
        brakeDoubleSolenoid.set(Value.kForward);
    }

    public void brakeDisable() {
        brakeDoubleSolenoid.set(Value.kReverse);
    }

    public void setFinalTargetPositions(ArmSetpoint armSetpoint) {
        armRotationFinalTarget = armSetpoint.getRotationSetpoint();
        armExtensionFinalTarget = armSetpoint.getExtensionSetpoint();
    }

    public void updateFinalTargetPositions() {
        // Length only has one constraint (maximum) so just min the constraint and the target.
        armExtensionFinalTarget = Math.min(armExtensionFinalTarget, armPose.getArmLengthToHitConstraintNativeUnits());

        // Rotation is slightly trickier since it has constraints in either direction.
        // First, figure out which way the arm is rotating by checking the difference between the target and the actual.
        // Then, find the constraint that is in the proper direction relative to the current position out of the two constraints.
        // Remember to use max instead of min when looking at bounds that are lower than the current value.
        if (_armRotationPosition < armRotationFinalTarget) {
            // The arm is moving forward.
            armRotationFinalTarget = Math.min(armRotationFinalTarget, armPose.getArmRotationMaximumBoundNativeUnits());
        }
        else {
            // The arm is moving backward.
            armRotationFinalTarget = Math.max(armRotationFinalTarget, armPose.getArmRotationMinimumBoundNativeUnits());
        }
    }

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

    public void setArmRotationPosition(double setpoint, double rotationVelocity, double rotationAcceleration) {
        rotationMotorsLeader.configMotionCruiseVelocity(rotationVelocity, Constants.kTimeoutMs);
        rotationMotorsLeader.configMotionAcceleration(rotationAcceleration, Constants.kTimeoutMs);
        rotationMotorsLeader.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, rotationAFF);
        SmartDashboard.putNumber("ArmRotationPosition", setpoint);
    }

    public void setArmExtensionPosition(double setpoint, double extensionVelocity, double extensionAcceleration) {
        extensionMotor.configMotionCruiseVelocity(extensionVelocity, Constants.kTimeoutMs);
        extensionMotor.configMotionAcceleration(extensionAcceleration, Constants.kTimeoutMs);
        extensionMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, extensionAFF);
        SmartDashboard.putNumber("ArmExtensionPosition", setpoint);
    }

    public void stopArm() {
        brakeEnable();

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

    public double getCommandTimeoutSeconds() {
        double rotationDifference = Math.abs(rotationMotorsLeader.getSelectedSensorPosition() - armRotationFinalTarget);
        double extensionDifference = Math.abs(extensionMotor.getSelectedSensorPosition() - armExtensionFinalTarget);

        // Need to test actual motion times at speeds we like and then come up with a reasonable timeout formula.
        return 4;
        // double rotationTime = rotationDifference / ARM_ROTATION_TIMEOUT_ENCODER_TICKS_PER_SECOND;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeaderEncoderPosition", rotationMotorsLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("motor1position", rotationMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("motor2position", rotationMotor2.getSelectedSensorPosition());

        double setAngle = getAngleFromPosition((SmartDashboard.getNumber("ArmSetPosition", 0.0)));
        SmartDashboard.putNumber("SetPositonInDegrees", setAngle);

        double actualAngle = getAngleFromPosition((SmartDashboard.getNumber("LeaderEncoderPosition", 0.0)));
        SmartDashboard.putNumber("ActualPositionInDegrees", actualAngle);
        // Angle (in 360) = 360 * ((ArmSetPosition / CountsPerRevolution) % 360)
        // ArmSetPosition = (Angle/360) * CountsPerRevolution
        // from degrees to position is ~11.377778

        armPose.calculateInstantaneousMaximums();
        this.updateFinalTargetPositions();
        this.updateAFFs();
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
        return rotationMotorsLeader.getSelectedSensorPosition();
    }

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

    public double getPositionFromAngle(double intendedAngle) {
        double position = (intendedAngle/360) * Constants.COUNTS_PER_REVOLUTION;
          return position;
      }

    private void manageSetpoints() {
        int maxInstantaneousExtension = armPose.getArmLengthToHitConstraintNativeUnits();
        this.armExtensionInstantaneousTarget = Math.min(this.armExtensionFinalTarget, maxInstantaneousExtension);
    }

    private void setExtensionTarget(int encoderNativeUnits) {
        this.armExtensionFinalTarget = encoderNativeUnits;
    }

    private void setRotationTarget(int encoderNativeUnits) {
        this.armRotationFinalTarget = encoderNativeUnits;
    }
    public double getArmRotationInstantaneousTarget() {
        return armRotationInstantaneousTarget;
    }

    public double getArmExtensionInstantaneousTarget() {
        return armExtensionInstantaneousTarget;
    }
}
