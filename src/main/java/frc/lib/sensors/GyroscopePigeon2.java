package frc.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.lib.configs.appliers.Pigeon2ConfigApplier;

/** Pigeon 2 gyroscope */
public class GyroscopePigeon2 implements Gyroscope {
  
  private final Pigeon2 gyroscope;

  private final StatusSignal<Double> roll, pitch, yaw, rollVelocity, pitchVelocity, yawVelocity;

  public GyroscopePigeon2() {
    gyroscope = new Pigeon2(0);

    roll = gyroscope.getRoll();
    pitch = gyroscope.getPitch();
    yaw = gyroscope.getYaw();

    rollVelocity = gyroscope.getAngularVelocityXWorld();
    pitchVelocity = gyroscope.getAngularVelocityYWorld();
    yawVelocity = gyroscope.getAngularVelocityZWorld();
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10, roll, pitch, rollVelocity, pitchVelocity);

    Pigeon2ConfigApplier.applyFactoryDefault(gyroscope);
  }

  @Override
  public void getUpdatedVals(GyroscopeValues values) {
    values.rollRotations = Units.degreesToRotations(roll.getValue());
    values.pitchRotations = Units.degreesToRotations(pitch.getValue());
    values.yawRotations = Units.degreesToRotations(yaw.getValue());
    values.rollVelocityRotations = Units.degreesToRotations(rollVelocity.getValue());
    values.pitchVelocityRotations = Units.degreesToRotations(pitchVelocity.getValue());
    values.yawVelocityRotations = Units.degreesToRotations(yawVelocity.getValue());
  }

  @Override
  public void setYaw(double yawRotations) {
    gyroscope.setYaw(Units.rotationsToDegrees(yawRotations));
  }
}
