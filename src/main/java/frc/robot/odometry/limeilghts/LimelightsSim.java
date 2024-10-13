package frc.robot.odometry.limeilghts;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.sensor.GyroscopeIO.GyroscopeIOValues;
import frc.robot.LimelightHelpers;

public class LimelightsSim implements LimelightsIO {
  /** Limelight names */
  private final String[] names;

  /** Reference to pose estimator */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Initialize simulated limelights
   * 
   * @param names limelight names
   * @param poseEstimator reference to SwerveDrivePoseEstimator
   */
  public LimelightsSim(String[] names, SwerveDrivePoseEstimator poseEstimator) {
    this.names = names;
    this.poseEstimator = poseEstimator;
  }

  /** Essentially does nothing
   * 
   * @param gyroscopeValues
   * @param yaw
   */
  public void update(GyroscopeIOValues gyroscopeValues, Rotation2d yaw) {
    return;
  }
}

