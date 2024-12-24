package frc.robot.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Telemetry with shuffleboard */
public class Telemetry extends SubsystemBase {
  
  /** Telemetry singleton */
  private static Telemetry instance = null;

  /** Hashmaps of telemetry values for shuffleboard value suppliers */
  private HashMap<String, Double> doubles = new HashMap<String, Double>();
  private HashMap<String, Boolean> bools = new HashMap<String, Boolean>();

  /** Lists of value updating runnables */
  private ArrayList<Runnable> even_updaters = new ArrayList<Runnable>();
  private ArrayList<Runnable> odd_updaters = new ArrayList<Runnable>();

  /** Switches between 1 and 0 every periodic loop to switch between the two groups of updaters to be called */
  private int periodic_loop;

  /** Telemetry subsystem constructor */
  private Telemetry() {
    periodic_loop = 0;
  }

  /** Gets telemetry subsystem instance */
  public Telemetry getInstance() {
    if (instance == null) {
      instance = new Telemetry();
    }

    return instance;
  }

  /** Runs every periodic loop, updating telemetry values in the hashmaps */
  @Override
  public void periodic() {
    periodic_loop ^= 1;

    if (periodic_loop == 0) {
      for (Runnable runnable : even_updaters) {
        runnable.run();
      }
    } else {
      for (Runnable runnable : odd_updaters) {
        runnable.run();
      }
    }
  }

  /**
   * Gets the value of a double in the hashmap of doubles
   * 
   * @param name name of value in hashmap
   * @return the value of a double in the hashmap of doubles
   */
  public double getDouble(String name) {
    return doubles.getOrDefault(name, 0.0);
  }

  /**
   * Gets the value of a bool in the hashmap of bools
   * 
   * @param name name of value in hashmap
   * @return the value of a bool in the hashmap of bools
   */
  public boolean getBool(String name) {
    return bools.getOrDefault(name, false);
  }

  /**
   * Add a double value along with its updater
   * 
   * @param name name of double value
   * @param supplier function that supplies the double's updated value
   */
  public void addDouble(String name, Supplier<Double> supplier) {
    even_updaters.add(
      () -> {
        doubles.put(name, supplier.get());
      });
  }

  /**
   * Add a boolean value along with its updater
   * 
   * @param name name of the bool value
   * @param supplier function that supplies the bool's updated value
   */
  public void addBool(String name, Supplier<Boolean> supplier) {
    even_updaters.add(
      () -> {
        bools.put(name, supplier.get());
      });
  }

  /** Staggers the added updaters between list of updaters called on odd and even periodic loops */
  public void staggerUpdaters() {
    int split_index = even_updaters.size() / 2;

    odd_updaters = new ArrayList<Runnable>(even_updaters.subList(split_index, even_updaters.size()));
    even_updaters = new ArrayList<Runnable>(even_updaters.subList(0, split_index));
  }
}
