package frc.lib;

import java.util.Objects;

/** CAN id wrapper that allows simpler bus handling */
public record CAN(int id, String bus) {
  public CAN { // prevents null values for id and bus jsut in case, could be a big issue in hardware
    Objects.requireNonNull(id);
    Objects.requireNonNull(bus);
  }

  public CAN(int id) { // allows you to make CAN records with just an id, defaulting to bus ""
    this(id, "");
  }
}
