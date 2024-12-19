package frc.lib.controllers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.lib.CAN;

/** TalonFX used as a position controller */
public class PositionControllerTalonFXSteer implements PositionController {

  private final TalonFX motor;

  private final CANcoder azimuthEncoder;

  private final StatusSignal<Double> position, velocity, acceleration, volts, amps;

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  public PositionControllerTalonFXSteer(
      CAN steerCAN,
      CAN encoderCAN,
      boolean enableFOC) {
        
    motor = new TalonFX(steerCAN.id(), steerCAN.bus());

    azimuthEncoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    position = azimuthEncoder.getAbsolutePosition();

    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();

    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {

  }

  @Override
  public void update(PositionControllerValues values) {

  }

  @Override
  public void setPos(double posRotations) {

  }

  @Override
  public void setSetpoint(double posRotations, double velRotationsPerSec) {

  }
}