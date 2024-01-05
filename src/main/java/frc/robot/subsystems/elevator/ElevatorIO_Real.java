// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIO_Real implements ElevatorIO {

    private TalonFX elevatorMotor = new TalonFX(CANID.kElevator);

    private PositionVoltage positionVoltage = new PositionVoltage(0);

    public ElevatorIO_Real(){

        var elevatorMotorConfigurator = elevatorMotor.getConfigurator();
        var elevatorMotorConfigs = new TalonFXConfiguration();

        elevatorMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        elevatorMotorConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kCurrentLimit;
        elevatorMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kCurrentLimit;
        elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        elevatorMotorConfigs.Slot0.kP = ElevatorConstants.kP;
        elevatorMotorConfigs.Slot0.kI = ElevatorConstants.kI;
        elevatorMotorConfigs.Slot0.kD = ElevatorConstants.kD;
        elevatorMotorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorConfigs.Slot0.kG = ElevatorConstants.kG;

        elevatorMotorConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.kSensorToVericalInches;

        var positionSignal = elevatorMotor.getPosition();
        var velocitySignal = elevatorMotor.getVelocity();
        var dutyCycleSignal = elevatorMotor.getDutyCycle();
        var tempSignal = elevatorMotor.getDeviceTemp();

        positionSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
        velocitySignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
        dutyCycleSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
        tempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency/4);

        elevatorMotorConfigurator.apply(elevatorMotorConfigs);
        elevatorMotor.optimizeBusUtilization();

        elevatorMotor.setPosition(ElevatorConstants.kMinHeight);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentHeightInches = getElevatorHeight();
        inputs.motorCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVoltage = elevatorMotor.getDutyCycle().getValueAsDouble() * 12;
        inputs.motorTemperature = elevatorMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void run(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
        positionVoltage.Position = setpoint;
        elevatorMotor.setControl(positionVoltage);
    }

    public double getElevatorHeight(){
        return elevatorMotor.getPosition().getValueAsDouble() + ElevatorConstants.kMinHeight;
    }

}
