package frc.robot.subsystems.implementations.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.config.game.rebuilt2026.tunerConstants.TunerConstants;
import frc.robot.io.interfaces.DriveIO;
import frc.robot.io.interfaces.DriveIOInputsAutoLogged;
import frc.robot.io.interfaces.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.implementations.drive.generated.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// A wrapper of the DriveBase around the generated CommandSwerveDrivetrain subsystem from the CTRE
// Phoenix Generated Project
public class DriveSwerveCTRE extends DriveBase {

  // drivetrain, MaxSpeed, MaxAngularRate all require things from a TunerConstants
  private final CommandSwerveDrivetrain drivetrain;
  private double MaxSpeed;
  private double MaxAngularRate;

  @AutoLogOutput private boolean fieldOrientedDrive = true;
  // The field and robot centric swerve request
  private final SwerveRequest.FieldCentric driveFieldCentric;
  private final SwerveRequest.RobotCentric driveRobotCentric;

  DriveIO io = new DriveIO();
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final ModuleIOInputsAutoLogged[] moduleInputs = {
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged()
  };

  public DriveSwerveCTRE(TunerConstants tunerConstants) {
    super("CTRE");
    drivetrain = tunerConstants.createDrivetrain();
    MaxSpeed =
        tunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    MaxAngularRate =
        RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    driveFieldCentric =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    driveRobotCentric =
        new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  }

  @Override
  public void runVelocity(ChassisSpeeds velocity) {
    if (fieldOrientedDrive) {
      drivetrain.setControl(
          driveFieldCentric
              .withVelocityX(velocity.vxMetersPerSecond)
              .withVelocityY(velocity.vyMetersPerSecond)
              .withRotationalRate(velocity.omegaRadiansPerSecond));
    } else {
      drivetrain.setControl(
          driveRobotCentric
              .withVelocityX(velocity.vxMetersPerSecond)
              .withVelocityY(velocity.vyMetersPerSecond)
              .withRotationalRate(velocity.omegaRadiansPerSecond));
    }
  }

  @Override
  public double getMaxLinearSpeed() {
    return MaxSpeed;
  }

  @Override
  public double getMaxAngularSpeed() {
    return MaxAngularRate;
  }

  @Override
  public void setFieldOrientedDrive(boolean enable) {
    fieldOrientedDrive = enable;
  }

  @Override
  public boolean isFieldOrientedDrive() {
    return fieldOrientedDrive;
  }

  @Override
  public void resetOdometry() {
    drivetrain.resetPose(new Pose2d());
  }

  @Override
  public void setPoseToMatchField() {
    // unsure what to do here
  }

  @Override
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  @Override
  public void setPose(Pose2d pose) {
    drivetrain.resetPose(pose);
  }

  @Override
  public double getAngle() {
    return drivetrain.getState().Pose.getRotation().getDegrees();
  }

  @Override
  public void lockPose() {
    SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    drivetrain.setControl(xBrake);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, moduleInputs, drivetrain);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Modules/frontleft", moduleInputs[0]);
    Logger.processInputs("Drive/Modules/frontright", moduleInputs[1]);
    Logger.processInputs("Drive/Modules/backleft", moduleInputs[2]);
    Logger.processInputs("Drive/Modules/backright", moduleInputs[3]);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    if (visionMeasurementStdDevs != null) {
      drivetrain.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    } else {
      drivetrain.addVisionMeasurement(robotPose, timestamp);
    }
  }
}
