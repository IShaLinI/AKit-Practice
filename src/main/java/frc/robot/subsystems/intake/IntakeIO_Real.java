// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIO_Real implements IntakeIO{

    private TalonFX intakeMotor = new TalonFX(CANID.kIntake);
    private VoltageOut voltageOut = new VoltageOut(0);

    public IntakeIO_Real() {
        var intakeMotorConfigurator = intakeMotor.getConfigurator();
        var intakeMotorConfigs = new TalonFXConfiguration();

        intakeMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeMotorConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.kCurrentLimit;
        intakeMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotorConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kCurrentLimit;
        intakeMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        var dutyCycleSignal = intakeMotor.getDutyCycle();
        var tempSignal = intakeMotor.getDeviceTemp();
        var currentSignal = intakeMotor.getSupplyCurrent();

        dutyCycleSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
        tempSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency / 4);
        currentSignal.setUpdateFrequency(CodeConstants.kMainLoopFrequency);

        intakeMotorConfigurator.apply(intakeMotorConfigs);
        intakeMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorTemperature = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getDutyCycle().getValueAsDouble() * 12;
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void run(double voltage) {
        voltageOut.Output = voltage;
        intakeMotor.setControl(voltageOut);
    }

}
