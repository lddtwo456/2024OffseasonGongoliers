package frc.robot.odometry.targetting;

import edu.wpi.first.math.geometry.Pose2d;

public interface FieldTargetsSupplier {
  public double getYawToSpeaker(Pose2d currentPosition);
}
