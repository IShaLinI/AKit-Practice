package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIO_Sim implements IntakeIO {

  private double voltage = 0;

  private DCMotorSim intakeMotor =
      new DCMotorSim(DCMotor.getFalcon500(1), 1 / IntakeConstants.kGearing, 0.005);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(1 / CodeConstants.kMainLoopFrequency);
    inputs.motorCurrent = intakeMotor.getCurrentDrawAmps();
    inputs.motorVoltage = voltage;
    inputs.motorTemperature = 0;
  }

  @Override
  public void run(double voltage) {
    this.voltage = voltage;
    intakeMotor.setInputVoltage(voltage);
  }
}
