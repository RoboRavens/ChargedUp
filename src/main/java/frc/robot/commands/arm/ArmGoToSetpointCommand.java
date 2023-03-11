// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.util.arm.ArmMaximumConstraint;
import frc.util.arm.ArmPose;
import frc.util.arm.ArmSetpoint;

public class ArmGoToSetpointCommand extends CommandBase {
  private ArrayList<ArmSetpoint> subSetpoints = new ArrayList<ArmSetpoint>();
  private ArmSubsystem arm = Robot.ARM_SUBSYSTEM;
  private ArmSetpoint finalSetpoint;
  private int setpointIterator = 0;
  private Timer timer = new Timer();
  private double timeoutSeconds = 0;
  private ArmSetpoint currentSubSetpoint;

  public ArmGoToSetpointCommand(ArmSetpoint setpoint) {
    this.finalSetpoint = setpoint;
    this.currentSubSetpoint = setpoint;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.brakeDisable();
    System.out.println("default position command run");
    subSetpoints = new ArrayList<ArmSetpoint>();
    setpointIterator = 0;
    calculateSubSetpoints();
    setArmTargets();
    
    timeoutSeconds = arm.getCommandTimeoutSeconds();

    arm.motionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.brakeDisable();

    if (setpointIsFinished(currentSubSetpoint)) {
      setpointIterator++;
    }

    setArmTargets();

    arm.setArmRotationPosition(arm.getArmRotationInstantaneousTarget(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    arm.setArmExtensionPosition(arm.getArmExtensionInstantaneousTarget(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
  }

  private void calculateSubSetpoints() {
    // Sub-setpoints can be needed if the destination is in a different quadrant than the origin point.
    // The arm will not extend or rotate if that would put it in an illegal state,
    // but it will not automatically contract without a setpoint.
    // If the arm MUST contract in order to reach its destination,
    // this therefore will require an additional setpoint.
    // An example of this is if the arm is at max extension towards the corner of a quadrant,
    // and wants to go to max extension in the corner of a different quadrant.
    // It must first pass through the border of the two quadrants, which has a shorter max extension.
    double currentAngleDegrees = arm.getCurrentAngleDegrees();
    double setpointAngleDegress = finalSetpoint.getRotationSetpointDegrees();
    double rotationDifferenceDegress = setpointAngleDegress - currentAngleDegrees;
    double rotationDifferenceDegressAbsValue = Math.abs(rotationDifferenceDegress);
    boolean rotatingForward = (rotationDifferenceDegress > 0);

    // The default value for this variable is true IF the arm is rotating backwards.
    // The 360 is added so as to not invert the angle direction if the arm is backwards.
    double distanceToFirstQuadrantBoundary = (currentAngleDegrees + 360) % 90;

    // But if it's rotating forward, invert the distance.
    if (rotatingForward) {
      distanceToFirstQuadrantBoundary = 90 - distanceToFirstQuadrantBoundary;
    }

    // Count the number of boundaries that will be crossed.
    int quadrantBoundariesToCross = 0;

    // The difference between "no boundaries" and "any boundaries" depends on how far the
    // starting position is from a boundary, but subsequent boundaries are always 90 degree increments.
    if (rotationDifferenceDegressAbsValue > distanceToFirstQuadrantBoundary) {
      quadrantBoundariesToCross++;

      // Do not use the distance "consumed" by the first boundary in subsequent calculations.
      double marginalDistanceAfterFirstBoundary = rotationDifferenceDegressAbsValue - distanceToFirstQuadrantBoundary;

      // If the final destination is past a boundary, simply using integer division will truncate the remainder.
      quadrantBoundariesToCross += (int) marginalDistanceAfterFirstBoundary / 90;
    }
    
    if (quadrantBoundariesToCross > 0) {
      for (int i = 0; i < quadrantBoundariesToCross; i++) {
        double subSetpointExtension = 0;
        double subSetpointRotation = 0;

        if (rotatingForward) {
          subSetpointRotation = currentAngleDegrees + distanceToFirstQuadrantBoundary + (90 * i);
        }
        else {
          subSetpointRotation = currentAngleDegrees - distanceToFirstQuadrantBoundary - (90 * i);
        }

        ArmMaximumConstraint constraint = ArmPose.calculateMaxExtensionAtAngleDegrees(subSetpointRotation);
        subSetpointRotation = Math.min(finalSetpoint.getExtensionSetpoint(), constraint.getArmLengthToHitConstraintNativeUnits());
        
        ArmSetpoint boundarySubSetpoint = new ArmSetpoint("Boundary SubSetpoint " + i + 1, subSetpointExtension, subSetpointRotation);

        subSetpoints.add(boundarySubSetpoint);
      }
    }
  }

  public void setArmTargets() {
    // If there sub-setpoints remaining, set the target to be the current point.
    // Otherwise just set to the overall setpoint.
    // No points at all is equivalent to none remaining.
    if (subSetpoints.size() > setpointIterator) {
      arm.setFinalTargetPositions(subSetpoints.get(setpointIterator));
      currentSubSetpoint = subSetpoints.get(setpointIterator);
    }
    else {
      arm.setFinalTargetPositions(finalSetpoint);
    }
  }

  public boolean setpointIsFinished(ArmSetpoint setpoint) {
    double rotationError = Math.abs(setpoint.getRotationSetpoint() - arm.getCurrentRotationNativeUnits());
    double extensionError = Math.abs(setpoint.getExtensionSetpoint() - arm.getCurrentExtensionNativeUnits());

    boolean rotationIsFinished = rotationError < Constants.ARM_ROTATION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;
    boolean extensionIsFinished = extensionError < Constants.ARM_EXTENSION_IS_AT_SETPOINT_MARGIN_ENCODER_TICKS;

    return rotationIsFinished && extensionIsFinished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Note that terminating this command early, for example due to a timeout,
    // does not change the target of the arm, so if the next command issued
    // continues moving the arm to its target, it will do so.
    // However, if this command was processing a multi-setpoint path,
    // the target after this command terminates will be whatever sub-setpoint it was on,
    // which was not necessarily the final target.
    boolean timeout = timer.get() >= timeoutSeconds;
    return setpointIsFinished(finalSetpoint) || timeout;
  }
}
