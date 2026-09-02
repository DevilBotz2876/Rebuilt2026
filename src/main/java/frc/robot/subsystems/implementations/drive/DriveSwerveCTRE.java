package frc.robot.subsystems.implementations.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.config.game.rebuilt2026.tunerConstants.TunerConstants;
import frc.robot.io.interfaces.DriveIO;
import frc.robot.io.interfaces.DriveIOInputsAutoLogged;
import frc.robot.io.interfaces.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.implementations.drive.generated.CommandSwerveDrivetrain;
import java.util.Properties;
import org.littletonrobotics.junction.Logger;

// A wrapper of the DriveBase around the generated CommandSwerveDrivetrain subsystem from the CTRE
// Phoenix Generated Project
public class DriveSwerveCTRE extends DriveBase {

  public static class DriveSettings {
    public double maxSpeedMetersPerSecond;
    public double maxAngularRateRadiansPerSecond;

    public static DriveSettings getDriveSettings(Properties properties) {
      DriveSettings settings = new DriveSettings();
      settings.maxSpeedMetersPerSecond =
          Double.parseDouble(properties.getProperty("robot.drive.maxSpeedMetersPerSecond"));
      settings.maxAngularRateRadiansPerSecond =
          RotationsPerSecond.of(
                  Double.parseDouble(
                      properties.getProperty("robot.drive.maxAngularRateRotationsPerSecond")))
              .in(RadiansPerSecond);

      return settings;
    }
  }

  private final CommandSwerveDrivetrain drivetrain;
  private DriveSettings settings;

  private boolean fieldOrientedDrive = true;
  private double defenseSpeedFactor = 1.0;
  // The field and robot centric swerve request
  private final SwerveRequest.FieldCentric driveFieldCentric;
  private final SwerveRequest.RobotCentric driveRobotCentric;
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds; // the speeds for pathplanner
  private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();

  DriveIO io = new DriveIO();
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final ModuleIOInputsAutoLogged[] moduleInputs = {
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged()
  };

  public DriveSwerveCTRE(TunerConstants tunerConstants, DriveSettings settings) {
    super("CTRE");
    drivetrain = tunerConstants.createDrivetrain();
    drivetrain.getModule(2).getSteerMotor().setPosition(Angle.ofBaseUnits(0, Degree));
    this.settings = settings;

    
    // Ensure max speed is less than or equal to max speed at 12v
    this.settings.maxSpeedMetersPerSecond =
        Math.min(
            this.settings.maxSpeedMetersPerSecond,
            tunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    driveFieldCentric =
        new SwerveRequest.FieldCentric()
            .withDeadband(this.settings.maxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                this.settings.maxAngularRateRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    driveRobotCentric =
        new SwerveRequest.RobotCentric()
            .withDeadband(this.settings.maxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                this.settings.maxAngularRateRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // SETUP PATHPLANNER
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    if (config != null) {
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
          // starting pose)
          () -> drivetrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> {
            drivetrain.setControl(
                pathApplyRobotSpeeds
                    .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
          }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(15.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
              ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    }
    SmartDashboard.putData(
        getName() + "/Sysid/Dyanmic Forward", drivetrain.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData(
        getName() + "/Sysid/Dyanmic Reverse", drivetrain.sysIdDynamic(Direction.kReverse));
    SmartDashboard.putData(
        getName() + "/Sysid/Quasistatic Forward", drivetrain.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        getName() + "/Sysid/Quasistatic Reverse", drivetrain.sysIdQuasistatic(Direction.kReverse));
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
    return this.settings.maxSpeedMetersPerSecond;
  }

  @Override
  public double getMaxAngularSpeed() {
    return this.settings.maxAngularRateRadiansPerSecond;
  }

  @Override
  public void setFieldOrientedDrive(boolean enable) {
    fieldOrientedDrive = enable;
  }

  @Override
  public boolean isFieldOrientedDrive() {
    return fieldOrientedDrive;
  }

  public double getDriveSpeedFactor() {
    return defenseSpeedFactor;
  }

  public void setDriveSpeedFactor(double factor) {
    defenseSpeedFactor = factor;
  }

  @Override
  public void resetOdometry() {
    resetOdometry(new Pose2d());
  }

  public void resetOdometry(Pose2d pose) {
    drivetrain.resetPose(pose);
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
  public Command lockPose() {
    return drivetrain.applyRequest(() -> xBrake);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, moduleInputs, drivetrain);
    Logger.processInputs("Drive", inputs);
    Logger.recordOutput("Drive/isFieldOriented", fieldOrientedDrive);
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

  // Reset the field-centric heading
  @Override
  public Command resetFieldCentricHeading() {
    return drivetrain.runOnce(drivetrain::seedFieldCentric);
  }

  public void setMaxSpeed(double linearMPS, double angularRPS) {
    this.settings.maxAngularRateRadiansPerSecond = Units.rotationsToRadians(angularRPS);
    this.settings.maxSpeedMetersPerSecond = linearMPS;
    driveFieldCentric
        .withDeadband(this.settings.maxSpeedMetersPerSecond * 0.1)
        .withRotationalDeadband(
            this.settings.maxAngularRateRadiansPerSecond * 0.1); // Add a 10% deadband

    driveRobotCentric
        .withDeadband(this.settings.maxSpeedMetersPerSecond * 0.1)
        .withRotationalDeadband(
            this.settings.maxAngularRateRadiansPerSecond * 0.1); // Add a 10% deadband
  }
}
