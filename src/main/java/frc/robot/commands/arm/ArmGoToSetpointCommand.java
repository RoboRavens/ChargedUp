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
import frc.util.ArmSetpoint;

public class ArmGoToSetpointCommand extends CommandBase {
  private ArrayList<ArmSetpoint> subSetpoints;
  private ArmSubsystem arm = Robot.ARM_SUBSYSTEM;
  private ArmSetpoint setpoint;
  private Timer timer = new Timer();
  private double timeoutSeconds = 0;


  public ArmGoToSetpointCommand(ArmSetpoint setpoint) {
    this.setpoint = setpoint;
    addRequirements(arm);

    // Sub-setpoints can be needed if the destination is in a different quadrant than the origin point.
    // The arm will not extend or rotate if that would put it in an illegal state,
    // but it will not automatically contract without a setpoint.
    // If the arm MUST contract in order to reach its destination,
    // this therefore will require an additional setpoint.
    // An example of this is if the arm is at max extension towards the corner of a quadrant,
    // and wants to go to max extension in the corner of a different quadrant.
    // It must first pass through the border of the two quadrants, which has a shorter max extension.
    double currentAngleDegrees = Robot.ARM_SUBSYSTEM.getCurrentAngleDegrees();
    double setpointAngleDegress = setpoint.getRotationSetpointDegrees();
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

    int quadrantBoundariesToCross = 0;
    
    if (rotationDifferenceDegressAbsValue > distanceToFirstQuadrantBoundary) {
      quadrantBoundariesToCross++;

      double marginalDistanceAfterFirstBoundary = rotationDifferenceDegressAbsValue - distanceToFirstQuadrantBoundary;

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
        
        ArmSetpoint boundarySubSetpoint = new ArmSetpoint("Boundary SubSetpoint " + i + 1, subSetpointExtension, subSetpointRotation);
      }
    }




    //int quadrantBoundariesToCross = (int) setpoint.getRotationSetpointDegrees() / 90;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("default position command run");
    Robot.ARM_SUBSYSTEM.setFinalTargetPositions(setpoint);
    Robot.ARM_SUBSYSTEM.motionMagic();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ARM_SUBSYSTEM.setArmRotationPosition(arm.getArmRotationInstantaneousTarget(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
    Robot.ARM_SUBSYSTEM.setArmExtensionPosition(arm.getArmExtensionInstantaneousTarget(), Constants.ARM_ROTATION_VELOCITY, Constants.ARM_ROTATION_ACCELERATION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
