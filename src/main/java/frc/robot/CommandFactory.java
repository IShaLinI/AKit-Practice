package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.elevator.Elevator;

public class CommandFactory {

  // Subsystems
  MAXSwerve drivebase;
  Elevator elevator;

  /** Coordinates commands that involve multiple subsystems */
  public CommandFactory(MAXSwerve drivebase, Elevator elevator) {
    this.drivebase = drivebase;
    this.elevator = elevator;
  }


  public Command changeGamePiece(GamePiece gamePiece){
    return Commands.parallel(
      elevator.changeGamePiece(gamePiece)
    );
  }


}
