// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIO_Sim implements ElevatorIO {

  private double voltage = 0;

  private ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          1/ElevatorConstants.kGearing,
          4,
          Units.inchesToMeters(1.751),
          Units.inchesToMeters(0),
          Units.inchesToMeters(29),
          false,
          0);

  private PIDController elevatorController = new PIDController(1, 0, 0);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    elevatorSim.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.currentHeightInches = getElevatorHeight();
    inputs.motorCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.motorVoltage = voltage;
    inputs.motorTemperature = 0;
  }

  public double getElevatorHeight(){
    return (Units.metersToInches(elevatorSim.getPositionMeters()) * ElevatorConstants.kStages * Math.sin(ElevatorConstants.kAngle)) + ElevatorConstants.kMinHeight;
  }

  @Override
  public void run(double setpoint) {
    voltage = MathUtil.clamp(elevatorController.calculate(getElevatorHeight(), setpoint),-12,12);
    elevatorSim.setInputVoltage(voltage);
  }
}
