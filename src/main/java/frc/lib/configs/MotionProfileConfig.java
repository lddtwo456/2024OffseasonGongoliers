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
  
  /** Construct default config */
  public MotionProfileConfig() {
    this(0.0, 0.0);
  }

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
}
