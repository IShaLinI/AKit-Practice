package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ScoringSetpoint;
import frc.robot.Constants.ScoringSetpoints;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.Constants.IntakeConstants.IntakeDirection;

public class Intake extends SubsystemBase {
  
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private GamePiece gamePiece = GamePiece.CUBE;
  private ScoringSetpoint currentSetpoint = ScoringSetpoints.kZero;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);

    Logger.recordOutput("Intake/GamePiece", gamePiece.name());
    Logger.recordOutput("Intake/Setpoint", currentSetpoint.name());

  }

  public Command run(){
    return this.run(
      () -> {
        double voltage = (gamePiece == GamePiece.CUBE ? currentSetpoint.cubeSpeed()*12 : currentSetpoint.coneSpeed()*12);
        voltage *= (this.currentSetpoint.intakeDirection() == IntakeDirection.OUT ? 1 : -1);
        intakeIO.run(voltage);
      }
    );
  }

  public Command idle(){
    return this.run(
      () -> {
        intakeIO.run(IntakeConstants.kHoldingVoltage * -1);
      }
    );
  }

  public Command changeGamePiece(GamePiece gamePiece) {
    return this.runOnce(() -> this.gamePiece = gamePiece);
  }

  public Command changeSetpoint(ScoringSetpoint setpoint) {
    return this.runOnce(() -> this.currentSetpoint = setpoint);
  }

}
