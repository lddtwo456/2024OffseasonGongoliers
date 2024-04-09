package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Subsystem class for the shooter subsystem. */
public class Shooter extends Subsystem {

  /** Instance variable for the shooter subsystem singleton. */
  private static Shooter instance = null;

  /** Serializer. */
  private final VelocityControllerIO serializer;

  /** Serializer values. */
  private final VelocityControllerIOValues serializerValues;

  /** Flywheel. */
  private final VelocityControllerIO flywheel;

  /** Flywheel values. */
  private final VelocityControllerIOValues flywheelValues;

  private double serializerGoal, flywheelGoal;

  /** Creates a new instance of the shooter subsystem. */
  private Shooter() {
    serializer = ShooterFactory.createSerializer();
    serializerValues = new VelocityControllerIOValues();
    serializer.configure(SerializerConstants.CONTROLLER_CONSTANTS);

    flywheel = ShooterFactory.createFlywheel();
    flywheelValues = new VelocityControllerIOValues();
    flywheel.configure(FlywheelConstants.CONTROLLER_CONSTANTS);
  }

  /**
   * Gets the instance of the shooter subsystem.
   *
   * @return the instance of the shooter subsystem.
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() {
    serializer.update(serializerValues);
    flywheel.update(flywheelValues);

    flywheel.setSetpoint(flywheelGoal);
    serializer.setSetpoint(serializerGoal);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Serializer", serializerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Flywheel", flywheelValues);
  }

  public double getFlywheelVelocity() {
    return flywheelValues.velocityRotationsPerSecond;
  }

  public double getSerializerVelocity() {
    return serializerValues.velocityRotationsPerSecond;
  }

  public void setGoal(
      double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {
        this.flywheelGoal = flywheelVelocityRotationsPerSecond;
        this.serializerGoal = serializerVelocityRotationsPerSecond;
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        getFlywheelVelocity(),
        flywheelGoal,
        FlywheelConstants.SPEED_TOLERANCE) && 
    MathUtil.isNear(
        getSerializerVelocity(),
        serializerGoal,
        FlywheelConstants.SPEED_TOLERANCE);
  }
}
