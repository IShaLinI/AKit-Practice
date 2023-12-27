// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleInformation;
import frc.robot.Constants.MAXSwerveConstants;
import java.util.Queue;

/** Add your docs here. */
public class MAXSwerveIO_Real implements MAXSwerveIO {

  CANSparkMax driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  SparkMaxPIDController drivePID;
  SparkMaxPIDController turnPID;

  Queue<Double> drivePositionQueue;
  Queue<Double> turnPositionQueue;

  public MAXSwerveIO_Real(SwerveModuleInformation moduleInformation) {

    driveMotor = new CANSparkMax(moduleInformation.driveCAN_ID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(moduleInformation.turnCAN_ID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePID = driveMotor.getPIDController();
    turnPID = turnMotor.getPIDController();

    driveEncoder.setPositionConversionFactor(MAXSwerveConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(MAXSwerveConstants.kDriveEncoderVelocityFactor);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnEncoder.setPositionConversionFactor(MAXSwerveConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(MAXSwerveConstants.kTurnEncoderVelocityFactor);
    turnEncoder.setInverted(MAXSwerveConstants.kTurnEncoderInverted);
    turnEncoder.setAverageDepth(2);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(MAXSwerveConstants.kTurnEncoderPositionPIDMinInput);
    turnPID.setPositionPIDWrappingMaxInput(MAXSwerveConstants.kTurnEncoderPositionPIDMaxInput);

    drivePID.setP(MAXSwerveConstants.kDriveP);
    drivePID.setI(MAXSwerveConstants.kDriveI);
    drivePID.setD(MAXSwerveConstants.kDriveD);
    drivePID.setFF(MAXSwerveConstants.kDriveFF);
    drivePID.setOutputRange(MAXSwerveConstants.kDriveMinOutput, MAXSwerveConstants.kDriveMaxOutput);

    turnPID.setP(MAXSwerveConstants.kTurnP);
    turnPID.setI(MAXSwerveConstants.kTurnI);
    turnPID.setD(MAXSwerveConstants.kTurnD);
    turnPID.setFF(MAXSwerveConstants.kTurnFF);
    turnPID.setOutputRange(MAXSwerveConstants.kTurnMinOutput, MAXSwerveConstants.kTurnMaxOutput);

    driveMotor.setIdleMode(MAXSwerveConstants.kDriveIdleMode);
    turnMotor.setIdleMode(MAXSwerveConstants.kTurnIdleMode);

    driveMotor.setSmartCurrentLimit(MAXSwerveConstants.kDriveCurrentLimit);
    turnMotor.setSmartCurrentLimit(MAXSwerveConstants.kTurnCurrentLimit);

    driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kOdometryThreadFrequency));
    turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kOdometryThreadFrequency));

    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);

    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMPS = driveEncoder.getVelocity();

    inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = new Rotation2d(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    inputs.odometryDrivePositions =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveMPS(double mps) {
    drivePID.setReference(mps, ControlType.kVelocity);
  }

  @Override
  public void setTurnAngle(Rotation2d angle) {
    turnPID.setReference(angle.getRadians(), ControlType.kPosition);
  }
}
