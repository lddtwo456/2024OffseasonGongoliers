package frc.robot.odometry;

import frc.robot.RobotConstants;
import java.util.function.DoubleSupplier;

/** Simulated gyroscope. */
public class GyroscopeIOSim implements GyroscopeIO {

  /** Gyroscope's yaw. */
  private double yawRotations;

  private final DoubleSupplier yawVelocityRotationsPerSecondSupplier;

  /** Creates a new simulated gyroscope. */
  public GyroscopeIOSim() {
    yawRotations = 0.0;

    yawVelocityRotationsPerSecondSupplier =
        () -> Odometry.getInstance().getVelocity().getRotation().getRotations();
  }

  @Override
  public void configure() {}

  @Override
  public void update(GyroscopeIOValues values) {
    yawRotations +=
        yawVelocityRotationsPerSecondSupplier.getAsDouble() * RobotConstants.TICK_PERIOD;

    values.rollRotations = 0.0;
    values.pitchRotations = 0.0;
    values.yawRotations = yawRotations;
  }

  @Override
  public void setYaw(double yawRotations) {
    this.yawRotations = yawRotations;
  }
}
