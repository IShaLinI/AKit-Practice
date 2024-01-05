package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ScoringSetpoint;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class CommandFactory {

  // Subsystems
  MAXSwerve drivebase;
  Elevator elevator;
  Intake intake;

  /** Coordinates commands that involve multiple subsystems */
  public CommandFactory(MAXSwerve drivebase, Elevator elevator, Intake intake) {
    this.drivebase = drivebase;
    this.elevator = elevator;
    this.intake = intake;
  }


  public Command changeGamePiece(GamePiece gamePiece){
    return Commands.parallel(
      elevator.changeGamePiece(gamePiece),
      intake.changeGamePiece(gamePiece)
    );
  }

  public Command changeSetpoint(ScoringSetpoint setpoint){
    return Commands.parallel(
      elevator.changeSetpoint(setpoint),
      intake.changeSetpoint(setpoint)
    );
  }


}
