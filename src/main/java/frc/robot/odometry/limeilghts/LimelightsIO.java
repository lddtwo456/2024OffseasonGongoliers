package frc.robot.odometry.limeilghts;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.sensor.GyroscopeIO.GyroscopeIOValues;

public interface LimelightsIO {
  public void update(GyroscopeIOValues gyroscopeValues, Rotation2d yaw);
}
