package frc.lib.controllers.velocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.appliers.TalonFXConfigApplier;

/** Velocity controller using TalonFX and PID controller */
public class VelocityControllerTalonFXPIDF implements VelocityController {
  
  private final MechanismConfig config;

  private final TalonFX motor;

  private final StatusSignal<Double> position, velocity, acceleration, volts, amps;

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  /**
   * Creates a new TalonFX velocity controller with external PIDF
   * 
   * @param can can id and bus
   * @param config mechanism config
   * @param enableFOC true enables FOC (field oriented control)
   */
  public VelocityControllerTalonFXPIDF(
      CAN can,
      MechanismConfig config,
      boolean enableFOC) {
    
    this.config = config;

    motor = new TalonFX(can.id(), can.bus());

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();
    feedback = config.feedbackControllerConfig().createPIDController();

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
    BaseStatusSignal.setUpdateFrequencyForAll(10, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(motor);

    TalonFXConfigApplier.applyFactoryDefault(motor);
    TalonFXConfigApplier.apply(motor, config.motorConfig());
  }

  @Override
  public void getUpdatedVals(VelocityControllerValues values) {
    values.posRotations = position.getValue();
    values.velRotationsPerSec = velocity.getValue();
    values.accRotationsPerSecPerSec = acceleration.getValue();
    values.motorVolts = volts.getValue();
    values.motorAmps = amps.getValue();
  }

  @Override
  public void setPos(double positionRotations) {}

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double feedforwardVolts = feedforward.calculate(velocityRotationsPerSecond);

    double measuredVelocityRotationsPerSecond = velocity.getValue();

    double feedbackVolts = feedback.calculate(measuredVelocityRotationsPerSecond, velocityRotationsPerSecond);

    motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }
}
