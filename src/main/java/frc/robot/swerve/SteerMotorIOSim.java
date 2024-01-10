package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** Simulated steer motor. */
public class SteerMotorIOSim implements SteerMotorIO {

  /** Represents the motor used to steer the wheel. */
  private final DCMotor motorSim = DCMotor.getFalcon500(1); // TODO

  /** Represents the wheel steered by the motor. */
  private final FlywheelSim wheelSim =
      new FlywheelSim(motorSim, MK4iConstants.STEER_GEARING, MK4iConstants.STEER_MOI);

  /** Represents the position of the steer motor. */
  private double positionRotations;

  /** Feedback controller for the position. */
  private final PIDController positionFeedback = new PIDController(1.0, 0, 0);

  public SteerMotorIOSim() {
    positionRotations = 0.0;

    positionFeedback.enableContinuousInput(0, 1);
  }

  @Override
  public void configure() {}

  @Override
  public void update(SteerMotorIOValues values) {
    double voltage = positionFeedback.calculate(positionRotations);

    wheelSim.setInputVoltage(voltage);
    wheelSim.update(RobotConstants.TICK_PERIOD);

    double velocityRotationsPerSecond = wheelSim.getAngularVelocityRPM() / 60.0;
    positionRotations += velocityRotationsPerSecond;

    values.positionRotations = positionRotations;
    values.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }

  @Override
  public void setPosition(double positionRotations) {
    this.positionRotations = 0.0;
  }

  @Override
  public void setSetpoint(double positionRotations) {
    positionFeedback.setSetpoint(positionRotations);
  }
}
