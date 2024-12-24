package frc.robot.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Telemetry with shuffleboard */
public class Telemetry extends SubsystemBase {
  
  private static Telemetry instance = null;

  private HashMap<String, Double> doubles = new HashMap<String, Double>();
  private HashMap<String, Boolean> bools = new HashMap<String, Boolean>();

  private ArrayList<Runnable> even_updaters = new ArrayList<Runnable>();
  private ArrayList<Runnable> odd_updaters = new ArrayList<Runnable>();

  private int periodic_loop;

  private Telemetry() {
    periodic_loop = 0;
  }

  public Telemetry getInstance() {
    if (instance == null) {
      instance = new Telemetry();
    }

    return instance;
  }

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

  public double getDouble(String name) {
    return doubles.getOrDefault(name, 0.0);
  }

  public boolean getBool(String name) {
    return bools.getOrDefault(name, false);
  }

  public void addDouble(String name, Supplier<Double> supplier) {
    even_updaters.add(
      () -> {
        doubles.put(name, supplier.get());
      });
  }

  public void addBool(String name, Supplier<Boolean> supplier) {
    even_updaters.add(
      () -> {
        bools.put(name, supplier.get());
      });
  }

  public void staggerUpdaters() {
    int split_index = even_updaters.size() / 2;

    odd_updaters = new ArrayList<Runnable>(even_updaters.subList(split_index, even_updaters.size()));
    even_updaters = new ArrayList<Runnable>(even_updaters.subList(0, split_index));
  }
}
