package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Generic Elevator IO */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double currentHeightInches = 0.0;
    public double motorTemperature = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void run(double setpoint) {}
}
