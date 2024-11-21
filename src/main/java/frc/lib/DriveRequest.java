package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

public record DriveRequest(
  Translation2d translationAxis,
  Translation2d headingAxis,
  double rotationVelocityAxis) {

}
