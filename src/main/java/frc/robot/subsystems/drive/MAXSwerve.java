package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MAXSwerveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

public class MAXSwerve extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.kModuleLocations);

  private SwerveDrivePoseEstimator poseEstimator;

  private final MAXSwerveModule[] modules;

  private Rotation2d lastHeading = new Rotation2d();

  public MAXSwerve(GyroIO gyroIO, MAXSwerveIO[] MAXSwerveIOs) {
    this.gyroIO = gyroIO;

    modules = new MAXSwerveModule[MAXSwerveIOs.length];

    for (int i = 0; i < MAXSwerveIOs.length; i++) {
      modules[i] =
          new MAXSwerveModule(
              MAXSwerveIOs[i], DriveConstants.kIndexedSwerveModuleInformation[i].name + " Module");
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyroInputs.yawPosition, getModulePositions(), new Pose2d());
  }

  public void periodic() {

    lastHeading = getPose().getRotation();

    // Update Swerve Module Inputs
    for (var module : modules) {
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);

    var gyroDelta =
        new Rotation2d(
            kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                * 1
                / CodeConstants.kMainLoopFrequency);

    lastHeading = lastHeading.plus(gyroDelta);

    if (gyroInputs.connected) {
      poseEstimator.update(gyroInputs.yawPosition, getModulePositions());
    } else {
      poseEstimator.update(lastHeading, getModulePositions());
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }

  public Command runVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          // Calculate module setpoints
          ChassisSpeeds discreteSpeeds =
              ChassisSpeeds.discretize(speeds.get(), 1 / CodeConstants.kMainLoopFrequency);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(
              setpointStates, MAXSwerveConstants.kMaxDriveSpeed);

          // Send setpoints to modules
          SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
          for (int i = 0; i < modules.length; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].run(setpointStates[i]);
          }

          // Log setpoint states
          Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
          Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
        });
  }

  public Command stop() {
    return runVelocity(() -> new ChassisSpeeds());
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocity(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
  }

  public void runChassisSpeeds(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.discretize(speeds, 1/CodeConstants.kMainLoopFrequency);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAXSwerveConstants.kMaxDriveSpeed);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    for(int i = 0; i < modules.length; i++){
      optimizedSetpointStates[i] = modules[i].run(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getSwerveModuleState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if(RobotBase.isReal()){
      poseEstimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
    }else{
      poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
  }
}
