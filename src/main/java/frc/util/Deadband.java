package frc.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Deadband {
    public static Double adjustValueToZero(double value, double minimumValue) {
        if (value >= minimumValue || value <= (minimumValue * -1)) {
            return value;
        }
        
        return 0.0d;
    }

    public static void applySwerveRotationDeadband(SwerveModuleState[] desiredStates, SwerveModuleState[] hardwareStates) {
        // double deadbandValue = 0.12; // 0.0872665; // 5 degrees
        // var newFLSteer = new Rotation2d(Deadband.adjustValueRotation(states[0].angle.getRadians(), m_frontLeftModule.getSteerAngle(), deadbandValue));
        // SmartDashboard.putNumber("FLDB target/angle", desiredStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("FLDB target/speed", desiredStates[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("FLDB actual/angle", hardwareStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("FLDB actual/speed", hardwareStates[0].speedMetersPerSecond);
    
        // var deadbandStates = new SwerveModuleState[4];
        // deadbandStates[0] = Deadband.adjustValueRotation(desiredStates[0], hardwareStates[0], deadbandValue);
        //desiredStates[1] = Deadband.adjustValueRotation(desiredStates[1], hardwareStates[1], deadbandValue);
        //desiredStates[2] = Deadband.adjustValueRotation(desiredStates[2], hardwareStates[2], deadbandValue);
        //desiredStates[3] = Deadband.adjustValueRotation(desiredStates[3], hardwareStates[3], deadbandValue);
        // SmartDashboard.putNumber("FLDB DBS/angle", deadbandStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("FLDB DBS/speed", deadbandStates[0].speedMetersPerSecond);
    
        // var diff = desiredStates[0].angle.minus(hardwareStates[0].angle);
        // SmartDashboard.putNumber("diff", diff.getDegrees());
        // double diffRads = (diff.getRadians() + (Math.PI*2)) % Math.PI;
        // SmartDashboard.putNumber("diff2", Math.toDegrees(diffRads));
    
        //SmartDashboard.putBoolean("FLDB", desiredStates[0].angle.getRadians() == hardwareStates[0].angle.getRadians());
        //SmartDashboard.putBoolean("FRDB", desiredStates[1].angle.getRadians() == hardwareStates[1].angle.getRadians());
        //SmartDashboard.putBoolean("BLDB", desiredStates[2].angle.getRadians() == hardwareStates[2].angle.getRadians());
        //SmartDashboard.putBoolean("BRDB", desiredStates[3].angle.getRadians() == hardwareStates[3].angle.getRadians());
    }
    
    // if all 4 swerve modules have 0 velocity then stop modules from rotating
    public static void adjustRotationWhenStopped(SwerveModuleState[] desiredStates, SwerveModuleState[] hardwareStates) {
      for(int i = 0; i < 4; i++) {
        if (desiredStates[i].speedMetersPerSecond != 0) {
          return;
        }
      }

      for(int i = 0; i < 4; i++) {
        desiredStates[i].angle = hardwareStates[i].angle;
      }
    }
    /*
    private static SwerveModuleState adjustValueRotation(SwerveModuleState target, SwerveModuleState actual, double maxDifferenceInRadians) {
      double fullCircle = Math.PI*2;
      double directionOffset = 0;
  
      if (actual.speedMetersPerSecond < 0) {
          directionOffset = Math.PI;
      }
  
      double normalizedTarget = (target.angle.getRadians() + fullCircle + directionOffset) % fullCircle;
      double normalizedActual = (actual.angle.getRadians() + fullCircle) % fullCircle;
  
      double diff = normalizedActual - normalizedTarget;
      if (diff > Math.abs(maxDifferenceInRadians)) {
          return target;
      }
  
      return new SwerveModuleState(target.speedMetersPerSecond, new Rotation2d(target.angle.getRadians() + diff));
    }
    */
}