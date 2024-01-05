package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringSetpoint;
import frc.robot.Constants.ScoringSetpoints;
import frc.robot.Constants.ElevatorConstants.GamePiece;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  private ScoringSetpoint currentSetpoint = ScoringSetpoints.kCarry;

  @AutoLogOutput
  private GamePiece gamePiece = GamePiece.CUBE;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {

    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    double heightSetpoint = ((gamePiece == GamePiece.CUBE) ? currentSetpoint.cubeHeight() : currentSetpoint.coneHeight());

    elevatorIO.run(heightSetpoint);

    Logger.recordOutput("Elevator/CurrentHeight", elevatorInputs.currentHeightInches);
    Logger.recordOutput("Elevator/Setpoint", heightSetpoint);
    Logger.recordOutput("Elevator/Setpoint Name", currentSetpoint.name());

    set3DElevatorPoses();

  }

  public Command changeSetpoint(ScoringSetpoint setpoint) {
    return this.runOnce(() -> this.currentSetpoint = setpoint);
  }

  public Command changeGamePiece(GamePiece gamePiece) {
    return this.runOnce(() -> this.gamePiece = gamePiece);
  }

  private void set3DElevatorPoses(){

    double height = Units.inchesToMeters(elevatorInputs.currentHeightInches - 9.6);

    double motionRatio = Math.sin(Units.degreesToRadians(40))/ Math.sin(Units.degreesToRadians(50));

    double elevatorPoses[] = {
      0, 0, 0, 0, 0, 0, 0,
      (height*motionRatio)/2, 0, height/2, 0, 0, 0, 0,
      (height*motionRatio), 0, height, 0, 0, 0, 0,
    };

    Logger.recordOutput("Elevator/3D Poses", elevatorPoses);

  }


}
