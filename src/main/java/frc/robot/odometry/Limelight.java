package frc.robot.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.lib.sensor.GyroscopeIO.GyroscopeIOValues;
import frc.robot.LimelightHelpers;

public class Limelight {
  /** name of the limelight */
  private final String name;

  /** swerve pose estimator */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** initialize limelight */
  public Limelight(String name, SwerveDrivePoseEstimator poseEstimator) {
    this.name = name;

    this.poseEstimator = poseEstimator;
  }

  /** adds vision measuremnts to pose estimator each frame */
  public void update(GyroscopeIOValues gyroscopeValues) {
    // tell limelight yaw rotation (required for mt2)
    LimelightHelpers.SetRobotOrientation(this.name, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // get position estimate
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

    // don't update if measurement will likely be innacurate
    boolean rejectUpdate = false;

    if (Math.abs(gyroscopeValues.yawVelocityRotations) > 2) {
      rejectUpdate = true;
    } if (mt2.tagCount == 0) {
      rejectUpdate = true;
    } if (!rejectUpdate) {
      poseEstimator.addVisionMeasurement(
        mt2.pose, 
        mt2.timestampSeconds);
    }
  }
}
