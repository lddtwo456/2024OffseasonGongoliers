package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** Custom swerve module. */
public class SwerveModuleIOCustom implements SwerveModuleIO {

  /** Steer motor. */
  private final PositionControllerIO steerMotor;

  /** Steer motor values. */
  private final PositionControllerIOValues steerMotorValues = new PositionControllerIOValues();

  /** Drive motor. */
  private final VelocityControllerIO driveMotor;

  /** Drive motor values. */
  private final VelocityControllerIOValues driveMotorValues = new VelocityControllerIOValues();

  /** Speeds below this speed are zeroed. */
  private final double kSpeedDeadbandMetersPerSecond = 0.025;

  /** Module setpoint */
  private SwerveModuleState setpoint;

  /**
   * Creates a custom swerve module.
   *
   * @param config the swerve module's configuration.
   */
  public SwerveModuleIOCustom(SwerveModuleConfig config) {
    steerMotor = SwerveFactory.createSteerMotor(config);
    steerMotor.configure();

    driveMotor = SwerveFactory.createDriveMotor(config);
    driveMotor.configure();

    setpoint = new SwerveModuleState();
  }

  @Override
  public SwerveModuleState getState() {
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModuleState(
        driveMotorValues.velocityRotationsPerSecond * MK4iConstants.WHEEL_CIRCUMFERENCE,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }

  @Override
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  @Override
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy) {
    setpoint = optimize(setpoint, getState(), lazy);

    steerMotor.setSetpoint(setpoint.angle.getRotations(), 0);
    driveMotor.setSetpoint(setpoint.speedMetersPerSecond / MK4iConstants.WHEEL_CIRCUMFERENCE);

    this.setpoint = setpoint;
  }

  /**
   * Optimizes a swerve module's setpoint.
   *
   * @param setpoint the setpoint to optimize.
   * @param state the state of the module.
   * @param lazy if true, perform additional optimizations on the setpoint.
   * @return the optimized setpoint.
   */
  private SwerveModuleState optimize(
      SwerveModuleState setpoint, SwerveModuleState state, boolean lazy) {
    // Always perform this optimization, even when lazy
    setpoint = SwerveModuleState.optimize(setpoint, state.angle);

    // If we aren't lazy, don't perform additional optimizations
    if (!lazy) {
      return setpoint;
    }

    // Since we are lazy, perform additional optimizations

    // Deadband the module speed
    if (MathUtil.isNear(0.0, setpoint.speedMetersPerSecond, kSpeedDeadbandMetersPerSecond)) {
      setpoint.speedMetersPerSecond = 0.0;
    }

    // Keep previous angle if we aren't moving
    if (setpoint.speedMetersPerSecond == 0.0) {
      setpoint.angle = state.angle;
    }

    // Scale our speed by the module's error
    Rotation2d error = setpoint.angle.minus(state.angle);
    setpoint.speedMetersPerSecond *= error.getCos();

    return setpoint;
  }

  @Override
  public SwerveModulePosition getPosition() {
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModulePosition(
        driveMotorValues.positionRotations * MK4iConstants.WHEEL_CIRCUMFERENCE,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }
}
