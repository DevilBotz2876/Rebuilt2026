package frc.robot.io.interfaces;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
// TODO: Add Pathplanner back
// import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.implementations.drive.generated.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLog;

public class DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public Pose2d pose;
    public double poseX = 0.0;
    public double poseY = 0.0;
    public double poseRotInDegrees = 0.0;
    public Pose2d flippedPose;
    public double flippedPoseX = 0.0;
    public double flippedPoseY = 0.0;
    public double flippedPoseRotInDegrees = 0.0;
    public Translation3d currentAcceleration = new Translation3d();
  }

  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAcceleration = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    // public double driveStatorCurrentAmps = 0.0;
    // public double driveMotorStallCurrentAmps = 0.0;
    // public double driveTorqueCurrentCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public double steerPositionRad = 0.0;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAcceleration = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerSupplyCurrentAmps = 0.0;
    // public double steerStatorCurrentAmps = 0.0;
    // public double steerMotorStallCurrentAmps = 0.0;
    // public double steerTorqueCurrentCurrentAmps = 0.0;

    public boolean cancoderConnected = false;
    public double cancoderPositionRad = 0.0;
    public double cancoderAppliedVolts = 0.0;
    // public double cancoderCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(
      DriveIOInputs inputs, ModuleIOInputs[] moduleInputs, CommandSwerveDrivetrain drivetrain) {
    inputs.pose = drivetrain.getState().Pose;
    inputs.poseX = inputs.pose.getTranslation().getX();
    inputs.poseY = inputs.pose.getTranslation().getY();
    inputs.poseRotInDegrees = inputs.pose.getRotation().getDegrees();
    // TODO:Flip Pose
    // inputs.flippedPose = FlippingUtil.flipFieldPose(inputs.pose);
    // inputs.flippedPoseX = inputs.flippedPose.getTranslation().getX();
    // inputs.flippedPoseY = inputs.flippedPose.getTranslation().getY();
    // inputs.flippedPoseRotInDegrees = inputs.flippedPose.getRotation().getDegrees();

    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);
      ModuleIOInputs moduleInput = moduleInputs[i];

      moduleInput.cancoderPositionRad =
          Units.rotationsToRadians(module.getEncoder().getPosition().getValueAsDouble());
      moduleInput.cancoderConnected = module.getEncoder().isConnected();

      moduleInput.driveConnected = module.getDriveMotor().isConnected();
      moduleInput.drivePositionRad =
          Units.rotationsToRadians(module.getDriveMotor().getPosition().getValueAsDouble());
      moduleInput.driveVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(
              module.getDriveMotor().getVelocity().getValueAsDouble());
      moduleInput.driveAcceleration =
          module.getDriveMotor().getAcceleration().getValueAsDouble()
              * 2
              * Math.PI; // rotations * 2 * pi = rad
      moduleInput.driveAppliedVolts = module.getDriveMotor().getMotorVoltage().getValueAsDouble();
      moduleInput.driveSupplyCurrentAmps =
          module.getDriveMotor().getSupplyCurrent().getValueAsDouble();
      // moduleInput.driveStatorCurrentAmps =
      // module.getDriveMotor().getStatorCurrent().getValueAsDouble();
      // moduleInput.driveTorqueCurrentAmps =
      // module.getDriveMotor().getTorqueCurrent().getValueAsDouble();
      // moduleInput.driveMotorStallCurrentAmps =
      // module.getDriveMotor().getMotorStallCurrent().getValueAsDouble();

      moduleInput.steerConnected = module.getSteerMotor().isConnected();
      moduleInput.steerPositionRad =
          Units.rotationsToRadians(module.getSteerMotor().getPosition().getValueAsDouble());
      moduleInput.steerVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(
              module.getSteerMotor().getVelocity().getValueAsDouble());
      moduleInput.steerAcceleration =
          module.getSteerMotor().getAcceleration().getValueAsDouble()
              * 2
              * Math.PI; // rotations * 2 * pi = rad
      moduleInput.steerAppliedVolts = module.getSteerMotor().getMotorVoltage().getValueAsDouble();
      moduleInput.steerSupplyCurrentAmps =
          module.getSteerMotor().getSupplyCurrent().getValueAsDouble();
      // moduleInput.steerStatorCurrentAmps =
      // module.getSteerMotor().getStatorCurrent().getValueAsDouble();
      // moduleInput.steerTorqueCurrentAmps =
      // module.getSteerMotor().getTorqueCurrent().getValueAsDouble();
      // moduleInput.steerMotorStallCurrentAmps =
      // module.getSteerMotor().getMotorStallCurrent().getValueAsDouble();
    }
  }
  // Other methods for controlling the drive subsystem...
}
