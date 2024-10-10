package frc.robot.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.sensor.GyroscopeIO.GyroscopeIOValues;
import frc.robot.LimelightHelpers;

public class Limelights {
  /** Limelight names */
  private final String[] names;

  /** Reference to swerve pose estimator */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Initialize Limelights 
   * 
   * @param names List of the names of each limelight, used to communicate with them
   * @param poseEstimator Reference to the swerve drive pose estimator
  */
  public Limelights(String[] names, SwerveDrivePoseEstimator poseEstimator) {
    this.names = names;

    this.poseEstimator = poseEstimator;
  }

  /** Adds vision measurements to pose estimator each frame
   * 
   * @param gyroscopeValues Used to give the limelights a basic idea of how the robot is moving
   * @param yaw Used to give the limelight a field oriented yaw, where 0 faces the red alliance wall and positive is ccw
   */
  public void update(GyroscopeIOValues gyroscopeValues, Rotation2d yaw) {
    for (String limelight : this.names) {
      /** Tell limelights robot yaw rotation (required for mt2) */
      LimelightHelpers.SetRobotOrientation(limelight, yaw.getDegrees(), 0, 0, 0, 0, 0);

      /** Get vision pose estimate */
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

      /** Don't add vision measurement if it will likely be innacurate */
      if (gyroscopeValues.yawVelocityRotations > 2) {
        return;
      } if (mt2.tagCount == 0) {
        return;
      }

      /** Add vision measurement to swerve pose estimator */
      this.poseEstimator.addVisionMeasurement(
        mt2.pose, 
        mt2.timestampSeconds);
    }
  }
}
