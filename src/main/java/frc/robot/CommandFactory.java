package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.MAXSwerve;

public class CommandFactory {

    MAXSwerve drivebase;
    
    public CommandFactory(MAXSwerve drivebase) {
        
        this.drivebase = drivebase;

    }


    public Command followChoreoTrajectory(ChoreoTrajectory trajectory) {

        Pose2d initialPose = trajectory.getInitialPose();
        drivebase.setPose(initialPose);

        Logger.recordOutput("ChoreoTrajectory", trajectory.getPoses());

        return Choreo.choreoSwerveCommand(
            trajectory,
            drivebase::getPose, 
            AutoConstants.kSwerveController,
            drivebase::runChassisSpeeds,
            false,
            drivebase).andThen(drivebase::stop);

    }
    
}
