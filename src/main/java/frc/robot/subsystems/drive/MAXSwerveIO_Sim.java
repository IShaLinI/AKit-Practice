// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.MAXSwerveConstants;

public class MAXSwerveIO_Sim implements MAXSwerveIO {

  private DCMotorSim driveMotor =
      new DCMotorSim(DCMotor.getNEO(1), MAXSwerveConstants.kDriveMotorReduction, 0.025);
  private DCMotorSim turnMotor =
      new DCMotorSim(DCMotor.getNeo550(1), MAXSwerveConstants.kTurnMotorReduction, 0.025);

  private final Rotation2d turnAbsoluteInitialPosition =
      new Rotation2d(Math.random() * 2 * Math.PI); // Random initial position
  private double driveVolts = 0.0;
  private double turnVolts = 0.0;

  private PIDController turnController = new PIDController(1, 0.0, 0.0);
  private PIDController driveController = new PIDController(1.0, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, MAXSwerveConstants.kDriveFF);

  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    driveMotor.update(1 / CodeConstants.kMainLoopFrequency);
    turnMotor.update(1 / CodeConstants.kMainLoopFrequency);

    inputs.drivePositionMeters = driveMotor.getAngularPositionRad();
    inputs.driveVelocityMPS = driveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveVolts;
    inputs.driveCurrentAmps = driveMotor.getCurrentDrawAmps();

    inputs.turnPositionRad =
        new Rotation2d(turnMotor.getAngularPositionRad()).plus(turnAbsoluteInitialPosition);
    inputs.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnVolts;
    inputs.turnCurrentAmps = turnMotor.getCurrentDrawAmps();

    inputs.odometryDrivePositions = new double[] {inputs.drivePositionMeters};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPositionRad};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVolts = MathUtil.clamp(volts, -12, 12);
    driveMotor.setInputVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnVolts = MathUtil.clamp(volts, -12, 12);
    turnMotor.setInputVoltage(turnVolts);
  }

  @Override
  public void setDriveMPS(double mps) {
    setDriveVoltage(
        driveFeedforward.calculate(mps)
            + driveController.calculate(
                driveMotor.getAngularVelocityRadPerSec()
                    * MAXSwerveConstants.kWheelDiameterMeters
                    / 2,
                mps));
  }

  @Override
  public void setTurnAngle(Rotation2d angle) {
    setTurnVoltage(
        turnController.calculate(turnMotor.getAngularPositionRotations(), angle.getRotations()));
  }

  @Override
  public void resetDriveEncoder() {
    driveMotor.setState(0, 0);
  }
}
