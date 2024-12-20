package frc.lib.configs;

import java.time.format.FormatStyle;

/** 
 * Motor config
 * 
 * @param neutralBrake true means the motor will brake when given no voltage
 * @param ccwPositive true means positive voltage rotates the motor ccw
 * @param motorToMechRatio ratio of motor rotations to mechanism rotations
 * @param statorCurrentLimit current limit in the stator
 * @param supplyCurrentLimit supply current limit
 */
public record MotorConfig(
    boolean neutralBrake,
    boolean ccwPositive,
    double motorToMechRatio,
    double statorCurrentLimit,
    double supplyCurrentLimit) {

  /** Easier and more modular way to construct a motor config */
  public static class MotorBuilder {
    private boolean neutralBrake;
    private boolean ccwPositive;
    private double motorToMechRatio;
    private double statorCurrentLimit;
    private double supplyCurrentLimit;

    private MotorBuilder(
        boolean neutralBrake,
        boolean ccwPositive,
        double motorToMechRatio,
        double statorCurrentLimit,
        double supplyCurrentLimit) {
      this.neutralBrake = neutralBrake;
      this.ccwPositive = ccwPositive;
      this.motorToMechRatio = motorToMechRatio;
      this.statorCurrentLimit = statorCurrentLimit;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public static MotorBuilder defaults() {
      return new MotorBuilder(
        false, 
        true, 
        1.0, 
        80.0, 
        40.0);
    }

    public MotorBuilder neutralBrake(boolean neutralBrake) {
      this.neutralBrake = neutralBrake;
      return this;
    }

    public MotorBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public MotorBuilder motorToMechRatio(double motorToMechRatio) {
      this.motorToMechRatio = motorToMechRatio;
      return this;
    }

    public MotorBuilder statorCurrentLimit(double statorCurrentLimit) {
      this.statorCurrentLimit = statorCurrentLimit;
      return this;
    }

    public MotorBuilder supplyCurrentLimit(double supplyCurrentLimit) {
      this.supplyCurrentLimit = supplyCurrentLimit;
      return this;
    }

    public MotorConfig build() {
      return new MotorConfig(
        this.neutralBrake,
        this.ccwPositive,
        this.motorToMechRatio,
        this.statorCurrentLimit,
        this.supplyCurrentLimit);
    }
  }
}
