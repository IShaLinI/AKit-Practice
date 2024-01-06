package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Generic Intake IO */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double motorTemperature = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void run(double voltage) {}
}
