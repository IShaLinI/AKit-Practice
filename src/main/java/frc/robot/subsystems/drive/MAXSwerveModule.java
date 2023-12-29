// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveModule {

  private final MAXSwerveIO io;
  private final MAXSwerveIOInputsAutoLogged inputs = new MAXSwerveIOInputsAutoLogged();
  public final String name;

  private double lastPosition = 0.0;
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  public MAXSwerveModule(MAXSwerveIO io, String name) {
    this.io = io;
    this.name = name;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public SwerveModuleState run(SwerveModuleState desiredState) {
    var optimizedState = SwerveModuleState.optimize(desiredState, inputs.turnPositionRad);
    io.setDriveMPS(optimizedState.speedMetersPerSecond);
    io.setTurnAngle(optimizedState.angle);
    return optimizedState;
  }

  public void periodic() {
    Logger.processInputs("Swerve/" + name + " Module", inputs);
  }

  public void stop() {
    io.setDriveVoltage(0);
    io.setTurnVoltage(0);
  }

  public Rotation2d getTurnPosition() {
    return inputs.turnPositionRad;
  }

  public double getDrivePositionMeters() {
    return inputs.drivePositionMeters;
  }

  public double getDriveVelocityMPS() {
    return inputs.driveVelocityMPS;
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getTurnPosition());
  }

  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getDriveVelocityMPS(), getTurnPosition());
  }

  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }
}
