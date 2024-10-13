package frc.robot.odometry.targetting;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldTargetsSupplierRed implements FieldTargetsSupplier{
  public double getYawToSpeaker(Pose2d currentPosition) {
    return Math.atan2((5.565 - currentPosition.getY()), (15.545 - currentPosition.getX()));
  }
}
