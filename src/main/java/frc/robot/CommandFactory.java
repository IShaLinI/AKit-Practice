package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
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

    //Resets the pose of the robot to the initial pose of the trajectory
    Pose2d initialPose = trajectory.getInitialPose();
    drivebase.setPose(initialPose);

    //Logs the trajectory to be visualized in AScope
    Logger.recordOutput("ChoreoTrajectory", trajectory.getPoses());

    return Choreo.choreoSwerveCommand(
            trajectory,
            drivebase::getPose,
            AutoConstants.kSwerveController,
            drivebase::runChassisSpeeds,
            false,
            drivebase)
        .andThen(drivebase::stop);
  }
}
