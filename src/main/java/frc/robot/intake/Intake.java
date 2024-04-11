package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;
import frc.robot.intake.IntakeConstants.BackRollerConstants;
import frc.robot.intake.IntakeConstants.FrontRollerConstants;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Rollers. */
  private final VelocityControllerIO frontRoller, backRoller;

  /** Roller values. */
  private final VelocityControllerIOValues frontRollerValues, backRollerValues;

  private IntakeState setpoint, goal;

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    frontRoller = IntakeFactory.createFrontRoller();
    frontRollerValues = new VelocityControllerIOValues();
    frontRoller.configure(FrontRollerConstants.CONTROLLER_CONSTANTS);

    backRoller = IntakeFactory.createBackRoller();
    backRollerValues = new VelocityControllerIOValues();
    backRoller.configure(BackRollerConstants.CONTROLLER_CONSTANTS);

    setpoint = IntakeState.IDLE;
    goal = IntakeState.IDLE;
  }

  /**
   * Gets the instance of the intake subsystem.
   *
   * @return the instance of the intake subsystem.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  @Override
  public void periodic() {
    frontRoller.update(frontRollerValues);
    backRoller.update(backRollerValues);

    setpoint = goal;

    frontRoller.setSetpoint(setpoint.frontRollerVelocityRotationsPerSecond());
    backRoller.setSetpoint(setpoint.backRollerVelocityRotationsPerSecond());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    VelocityControllerIO.addToShuffleboard(tab, "Front Roller", frontRollerValues);
    VelocityControllerIO.addToShuffleboard(tab, "Back Roller", backRollerValues);
  }

  public Trigger noteStuck() {
    return new Trigger(() -> frontRollerStuck() || backRollerStuck());
  }

  private boolean frontRollerStuck() {
    return frontRollerValues.motorAmps > FrontRollerConstants.NOTE_AMPS;
  }

  private boolean backRollerStuck() {
    return backRollerValues.motorAmps > BackRollerConstants.NOTE_AMPS;
  }
  
  public IntakeState getState() {
    return new IntakeState(
        frontRollerValues.velocityRotationsPerSecond, backRollerValues.velocityRotationsPerSecond);
  }

  public void setGoal(IntakeState goal) {
    this.goal = goal;
  }

  public boolean atGoal() {
    return getState().at(goal);
  }
}
