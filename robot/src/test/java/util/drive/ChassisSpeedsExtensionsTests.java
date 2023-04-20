package util.drive;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.util.drive.ChassisSpeedsExtensions;

public class ChassisSpeedsExtensionsTests {
  @Test
  void BasicChassisSpeeds_WhenZero_ReturnsTrue() {
    var result = ChassisSpeedsExtensions.IsZero(new ChassisSpeeds(0, 0, 0));
    assertTrue(result);
  }

  @Test
  void FromRelativeSpeeds_WhenZero_ReturnsTrue() {
    var cs = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d(0));
    var result = ChassisSpeedsExtensions.IsZero(cs);
    assertTrue(result);
  }

  @Test
  void FromRelativeSpeeds_WhenZeroExceptForGyro_ReturnsTrue() {
    var cs = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d(1.1));
    var result = ChassisSpeedsExtensions.IsZero(cs);
    assertTrue(result);
  }

  @Test
  void BasicChassisSpeeds_WhenNotZero_ReturnsFalse() {
    var result = ChassisSpeedsExtensions.IsZero(new ChassisSpeeds(.5, .5, 0));
    assertFalse(result);
  }

  @Test
  void FromRelativeSpeeds_WhenNotZero_ReturnsFalse() {
    var cs = ChassisSpeeds.fromFieldRelativeSpeeds(.5, .5, 0, new Rotation2d(0));
    var result = ChassisSpeedsExtensions.IsZero(cs);
    assertFalse(result);
  }
}
