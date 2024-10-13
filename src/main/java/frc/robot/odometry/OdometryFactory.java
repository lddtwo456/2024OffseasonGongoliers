package frc.robot.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import frc.lib.sensor.GyroscopeIO;
import frc.lib.sensor.GyroscopeIOPigeon2;
import frc.lib.sensor.GyroscopeIOSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;
import frc.robot.odometry.limeilghts.Limelights;
import frc.robot.odometry.limeilghts.LimelightsIO;
import frc.robot.odometry.limeilghts.LimelightsSim;

/** Factory for creating odometry subsystem hardware. */
public class OdometryFactory {

  /**
   * Creates the gyroscope.
   *
   * @return the gyroscope.
   */
  public static GyroscopeIO createGyroscope(Odometry odometry) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ODOMETRY))
      return new GyroscopeIOPigeon2();

    return new GyroscopeIOSim(() -> Units.radiansToRotations(odometry.getVelocity().dtheta));
  }

  /**
   * Creates the limelights wrapper
   * 
   * @return limelights wrapper
   */
  public static LimelightsIO createLimelights(String[] names, SwerveDrivePoseEstimator poseEstimator) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.LIMELIGHT))
      return new Limelights(names, poseEstimator);

    return new LimelightsSim(names, poseEstimator);
  }
}
