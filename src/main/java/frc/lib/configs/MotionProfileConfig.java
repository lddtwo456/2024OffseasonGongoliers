package frc.lib.configs;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** 
 * Motion profile config
 * 
 * @param maxVelocity maximum velocity of the profile
 * @param maxAcceleration maximum acceleration of the profile
 */
public record MotionProfileConfig(
    double maxVelocity,
    double maxAcceleration) {

  /**
   * Calculates an acceleration using a ramp duration
   * 
   * @param maxSpeed max speed in units per second
   * @param desiredRampDurationSeconds desired duration to ramp from no speed to max speed
   * @return the desired acceleration in units per second per second
   */
  public static double calculateAcceleration(
      double maxSpeed, double desiredRampDurationSeconds) {
    
    return maxSpeed / desiredRampDurationSeconds;
  }

  /**
   * Creates a new velocity clamper using this motion profile config
   * 
   * @return a new velocity clamper using this motion profile config
   */
  public Function<Double, Double> createVelocityClamper() {
    return velocity -> MathUtil.clamp(velocity, -maxVelocity, maxVelocity);
  }

  /**
   * Creates a new acceleration (slew rate) limiter using this motion profile config
   * 
   * @return a new acceleration (slew rate) limiter using this motion profile config
   */
  public SlewRateLimiter createAccelerationLimiter() {
    return new SlewRateLimiter(maxAcceleration);
  }

  /**
   * Creates a new trapezoidal motion profile using this motion profile config
   * 
   * @return a new trapezoidal motion profile using this motion profile config
   */
  public TrapezoidProfile createTrapezoidProfile() {
    return new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
  }

  /** Easier and more modular way to construct motion profile configs */
  public static class MotionProfileBuilder {
    private double maxVelocity;
    private double maxAcceleration;

    private MotionProfileBuilder(
        double maxVelocity,
        double maxAcceleration) {
      this.maxVelocity = maxVelocity;
      this.maxAcceleration = maxAcceleration;
    }

    public static MotionProfileBuilder defaults() {
      return new MotionProfileBuilder(
        0.0, 
        0.0);
    }

    public MotionProfileBuilder maxVelocity(double maxVelocity) {
      this.maxVelocity = maxVelocity;
      return this;
    }

    public MotionProfileBuilder maxAcceleration(double maxAcceleration) {
      this.maxAcceleration = maxAcceleration;
      return this;
    }

    public MotionProfileConfig build() {
      return new MotionProfileConfig(
        this.maxVelocity,
        this.maxAcceleration);
    }
  }
}
