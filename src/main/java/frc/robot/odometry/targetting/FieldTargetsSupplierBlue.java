package frc.robot.odometry.targetting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldTargetsSupplierBlue implements FieldTargetsSupplier {
  public double getYawToSpeaker(Pose2d currentPosition) {
    return Math.atan2((5.565 - currentPosition.getY()), (0 - currentPosition.getX()));
  }
}
