package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.MAXSwerve;
import org.littletonrobotics.junction.Logger;


public class CommandFactory {

  //Subsystems
  MAXSwerve drivebase;
    /**
     * Controls the subsystems and provides commands for the robot
     */
  public CommandFactory(MAXSwerve drivebase) {
    this.drivebase = drivebase;
  }

  //Follow a choreo trajectory
  public Command followChoreoTrajectory(ChoreoTrajectory trajectory) {

    var trajectorySequence = new SequentialCommandGroup(
        new InstantCommand(() -> Logger.recordOutput("ChoreoTrajectory", trajectory.getPoses())),
        new InstantCommand(() -> drivebase.setPose(trajectory.getInitialPose()), drivebase),
        Choreo.choreoSwerveCommand(
            trajectory,
            drivebase::getPose,
            AutoConstants.kSwerveController,
            drivebase::runChassisSpeeds,
            false,
            drivebase),
        drivebase.stop()
    );
    
    return trajectorySequence;
  }

  public Command followChoreoTrajectory(ChoreoTrajectory trajectory, SequentialCommandGroup eventSequence){

    var trajectorySequence = new SequentialCommandGroup(
        new InstantCommand(() -> Logger.recordOutput("ChoreoTrajectory", trajectory.getPoses())),
        new InstantCommand(() -> drivebase.setPose(trajectory.getInitialPose()), drivebase),
        Choreo.choreoSwerveCommand(
            trajectory,
            drivebase::getPose,
            AutoConstants.kSwerveController,
            drivebase::runChassisSpeeds,
            false,
            drivebase),
        drivebase.stop()
    );

    return trajectorySequence.alongWith(eventSequence);

  }

  //Action Blocks
  public Command print(String message){
    return new InstantCommand(() -> System.out.println(message));
  }

  //Event Sequences
  public SequentialCommandGroup TestEvents(){
    return new SequentialCommandGroup(
        print("Start"),
        Commands.waitSeconds(1.5),
        print("Turning"),
         Commands.waitSeconds(1.5),
        print("Grabbed Piece")
    );
  }

}
