package frc.lib.configs;

import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * Absolute encoder config 
 * 
 * @param ccwPositive true means positive voltages turn the motor ccw
 * @param sensorToMechRatio ratio of sensor rotations to mechanism rotations
 * @param offset starting offset for encoder
 */
public record AbsoluteEncoderConfig(
    boolean ccwPositive, 
    double sensorToMechRatio, 
    Rotation2d offset) {

  /** Easier and more modular way to construct absolute encoder configs */
  public static class AbsoluteEncoderBuilder{
    private boolean ccwPositive;
    private double sensorToMechRatio;
    private Rotation2d offset;

    private AbsoluteEncoderBuilder(
        boolean ccwPositive,
        double sensorToMechRatio,
        Rotation2d offset) {
      this.ccwPositive = ccwPositive;
      this.sensorToMechRatio = sensorToMechRatio;
      this.offset = offset;
    }

    public static AbsoluteEncoderBuilder defaults() {
      return new AbsoluteEncoderBuilder(
        true, 
        1.0, 
        new Rotation2d());
    }

    public AbsoluteEncoderBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public AbsoluteEncoderBuilder sensorToMechRatio(double sensorToMechRatio) {
      this.sensorToMechRatio = sensorToMechRatio;
      return this;
    }

    public AbsoluteEncoderBuilder offset(Rotation2d offset) {
      this.offset =offset;
      return this;
    }

    public AbsoluteEncoderConfig build() {
      return new AbsoluteEncoderConfig(
        this.ccwPositive,
        this.sensorToMechRatio,
        this.offset);
    }
  }
}
