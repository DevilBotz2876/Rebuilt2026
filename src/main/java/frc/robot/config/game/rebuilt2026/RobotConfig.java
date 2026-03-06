package frc.robot.config.game.rebuilt2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.config.game.rebuilt2026.tunerConstants.TunerConstants;
import frc.robot.io.implementations.motor.MotorIOArmStub;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOElevatorStub;
import frc.robot.io.implementations.motor.MotorIOFlywheelStub;
import frc.robot.io.implementations.motor.MotorIOSparkMax;
import frc.robot.io.implementations.motor.MotorIOSparkMax.SparkMaxSettings;
import frc.robot.io.implementations.motor.MotorIOStub;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.controls.arm.IntakeArmControls;
import frc.robot.subsystems.controls.combination.AutoControls;
import frc.robot.subsystems.controls.combination.DriverControls;
import frc.robot.subsystems.controls.combination.AutoControls.AutoRoutineSettings;
import frc.robot.subsystems.controls.combination.DriverControls.DriverControlsSettings;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.controls.flywheel.ConveyorControls;
import frc.robot.subsystems.controls.flywheel.IntakeControls;
import frc.robot.subsystems.controls.flywheel.ShooterControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveCTRE;
import frc.robot.subsystems.implementations.drive.DriveSwerveCTRE.DriveSettings;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.implementations.motor.SimpleMotorSubsystem;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.implementations.vision.camera.CameraBase;
import frc.robot.subsystems.implementations.vision.camera.CameraPhoton;
import frc.robot.subsystems.implementations.vision.camera.CameraPhotonSim;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;
import frc.robot.subsystems.interfaces.Motor;
import frc.robot.subsystems.interfaces.SimpleMotor.SimpleMotorSettings;
import frc.robot.subsystems.interfaces.Vision.VisionSettings;
import java.util.Optional;
import frc.robot.util.Elastic;

import java.util.Properties;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public DriveBase drive;
  public SendableChooser<Command> autoChooser;
  public VisionSubsystem vision;
  public FlywheelMotorSubsystem intakeFlywheel;
  public FlywheelMotorSubsystem shooterFlywheel;
  public FlywheelMotorSubsystem indexerFlywheel;
  public FlywheelMotorSubsystem conveyorFlywheel;
  public ArmMotorSubsystem intakeArm;

  public Properties properties;
  // TODO: Add VisionSubsystem Declaration

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(Properties robotProperties) {
    if (robotProperties.containsKey("robot.drive")) {
      if (robotProperties.getProperty("robot.drive").equals("ctre")) {
        drive = new DriveSwerveCTRE(new TunerConstants(robotProperties), DriveSettings.getDriveSettings(robotProperties));
      }
    } else {
      drive = new DriveBase("Stub");
      Elastic.sendNotification( new Elastic.Notification().withDescription("USING DRIVE BASE. UNABLE TO RUN AUTO ROUTINES"));
    //   autoChooser = new SendableChooser<>();
    }

    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    }

    intakeFlywheel = createFlywheel(robotProperties, "intakeFlywheel");
    shooterFlywheel = createFlywheel(robotProperties, "shooterFlywheel");
    indexerFlywheel = createFlywheel(robotProperties, "indexerFlywheel");
    conveyorFlywheel = createFlywheel(robotProperties, "conveyorFlywheel");
    intakeArm = createArm(robotProperties, "intakeArm");

    vision = createVisionSubsystem(robotProperties);
    properties = robotProperties;
    if (robotProperties.containsKey("robot.drive")) {
      if (robotProperties.getProperty("robot.drive").equals("ctre")) {
        DriverControlsSettings driverSettings = DriverControlsSettings.getDriverControlsSettings(robotProperties);
        AutoRoutineSettings autoRoutineSettings = AutoRoutineSettings.getAutoRoutineSettings(robotProperties);
        AutoControls.registerNamedCommands(drive, intakeFlywheel, shooterFlywheel, indexerFlywheel, conveyorFlywheel, autoRoutineSettings, driverSettings);
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> {
        stream = stream.filter(auto -> auto.getName().startsWith(properties.getProperty("robot.name").toLowerCase() + "-"));
        return stream;
      }
        );
      }
    }
  }

  public void configureBindings() {
    DriverControlsSettings driverSettings = DriverControlsSettings.getDriverControlsSettings(properties);
    DriveControls.setupController(drive, mainController);

    IntakeControls.setupSpeedController(intakeFlywheel, mainController);
    ShooterControls.setupSpeedController(shooterFlywheel, indexerFlywheel, mainController);
    ConveyorControls.setupSpeedController(conveyorFlywheel, mainController);
    IntakeArmControls.setupController(intakeArm, mainController);

    DriverControls.setupMainController(
        drive,
        intakeFlywheel,
        shooterFlywheel,
        indexerFlywheel,
        conveyorFlywheel,
        intakeArm,
        mainController,
        driverSettings);

    DriverControls.setupFlywheelSmartDashboardControl(intakeFlywheel);
    DriverControls.setupFlywheelSmartDashboardControl(shooterFlywheel);
    DriverControls.setupFlywheelSmartDashboardControl(indexerFlywheel);
    DriverControls.setupFlywheelSmartDashboardControl(conveyorFlywheel);
    DriverControls.setupArmSmartDashboardControl(intakeArm);

    if (null != this.autoChooser) {
      SmartDashboard.putData("Autonomous", this.autoChooser);
    }
  }

  private void registerNamedCommands(DriverControlsSettings settings) {
    NamedCommands.registerCommand("Dynamic Drive to Hub (radius)", new WaitCommand(1.0));
    NamedCommands.registerCommand("Launch ", new WaitCommand(1.0));
  }

  private SimpleMotorSubsystem createSimpleMotor(Properties robotProperties, String name) {
    SimpleMotorSettings simpleMotorSettings = new SimpleMotorSettings();
    String simpleMotorSettingsPrefix = name + ".simpleMotorSettings";

    simpleMotorSettings.color =
        new Color8Bit(
            Integer.parseInt(robotProperties.getProperty(simpleMotorSettingsPrefix + ".color.red")),
            Integer.parseInt(
                robotProperties.getProperty(simpleMotorSettingsPrefix + ".color.green")),
            Integer.parseInt(
                robotProperties.getProperty(simpleMotorSettingsPrefix + ".color.blue")));

    simpleMotorSettings.maxVelocityInRadiansPerSecond =
        Double.parseDouble(
            robotProperties.getProperty(
                simpleMotorSettingsPrefix + ".maxVelocityInRadiansPerSecond"));
    simpleMotorSettings.maxAccelerationInRadiansPerSecondSquared =
        Double.parseDouble(
            robotProperties.getProperty(
                simpleMotorSettingsPrefix + ".maxAccelerationInRadiansPerSecondSquared"));

    simpleMotorSettings.targetPositionToleranceInRad =
        Double.parseDouble(
            robotProperties.getProperty(
                simpleMotorSettingsPrefix + ".targetPositionToleranceInRad"));
    simpleMotorSettings.maxPositionInRads =
        Double.parseDouble(
            robotProperties.getProperty(simpleMotorSettingsPrefix + ".maxPositionInRads"));
    simpleMotorSettings.minPositionInRads =
        Double.parseDouble(
            robotProperties.getProperty(simpleMotorSettingsPrefix + ".minPositionInRads"));
    simpleMotorSettings.startingPositionInRads =
        Double.parseDouble(
            robotProperties.getProperty(simpleMotorSettingsPrefix + ".startingPositionInRads"));
    simpleMotorSettings.moiKgMetersSquared =
        Double.parseDouble(
            robotProperties.getProperty(simpleMotorSettingsPrefix + ".moiKgMetersSquared"));

    simpleMotorSettings.feedforward =
        new SimpleMotorFeedforward(
            Double.parseDouble(
                robotProperties.getProperty(simpleMotorSettingsPrefix + ".feedforward.ks")),
            Double.parseDouble(
                robotProperties.getProperty(simpleMotorSettingsPrefix + ".feedforward.kv")),
            Double.parseDouble(
                robotProperties.getProperty(simpleMotorSettingsPrefix + ".feedforward.ka", "0.0")));

    simpleMotorSettings.motor =
        getDCMotor(robotProperties.getProperty(simpleMotorSettingsPrefix + ".DCMotor"));

    MotorIOBaseSettings IOSettings = getMotorIOBaseSettings(robotProperties, name);

    switch (robotProperties.getProperty(name + ".motor.motorController")) {
      case "talonFX":
        TalonFxSettings talonSettings = TalonFxSettings.getSettings(robotProperties, name);
        return new SimpleMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, simpleMotorSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = SparkMaxSettings.getSettings(robotProperties, name);
        return new SimpleMotorSubsystem(
            new MotorIOSparkMax(IOSettings, sparkMaxSettings), name, simpleMotorSettings);

      case "sim":
      default:
        return new SimpleMotorSubsystem(
            new MotorIOStub(IOSettings, simpleMotorSettings), name, simpleMotorSettings);
    }
  }

  private FlywheelMotorSubsystem createFlywheel(Properties robotProperties, String name) {
    FlywheelSettings flywheelSettings = new FlywheelSettings();
    String flywheelSettingsPrefix = name + ".flywheelSettings";

    flywheelSettings.color =
        new Color8Bit(
            Integer.parseInt(robotProperties.getProperty(flywheelSettingsPrefix + ".color.red")),
            Integer.parseInt(robotProperties.getProperty(flywheelSettingsPrefix + ".color.green")),
            Integer.parseInt(robotProperties.getProperty(flywheelSettingsPrefix + ".color.blue")));

    flywheelSettings.maxVelocityInRPMs =
        Double.parseDouble(
            robotProperties.getProperty(flywheelSettingsPrefix + ".maxVelocityInRPMs"));
    flywheelSettings.targetVelocityToleranceInRPMs =
        Double.parseDouble(
            robotProperties.getProperty(flywheelSettingsPrefix + ".targetVelocityToleranceInRPMs"));
    flywheelSettings.moiKgMetersSquared =
        Double.parseDouble(
            robotProperties.getProperty(flywheelSettingsPrefix + ".moiKgMetersSquared"));

    flywheelSettings.feedforward =
        new SimpleMotorFeedforward(
            Double.parseDouble(
                robotProperties.getProperty(flywheelSettingsPrefix + ".feedforward.ks")),
            Double.parseDouble(
                robotProperties.getProperty(flywheelSettingsPrefix + ".feedforward.kv")),
            Double.parseDouble(
                robotProperties.getProperty(flywheelSettingsPrefix + ".feedforward.ka", "0.0")));

    flywheelSettings.motor =
        getDCMotor(robotProperties.getProperty(flywheelSettingsPrefix + ".DCMotor"));

    MotorIOBaseSettings IOSettings = getMotorIOBaseSettings(robotProperties, name);

    switch (robotProperties.getProperty(name + ".motor.motorController")) {
      case "talonFX":
        TalonFxSettings talonSettings = TalonFxSettings.getSettings(robotProperties, name);
        return new FlywheelMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, flywheelSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = SparkMaxSettings.getSettings(robotProperties, name);
        return new FlywheelMotorSubsystem(
            new MotorIOSparkMax(IOSettings, sparkMaxSettings), name, flywheelSettings);

      case "sim":
      default:
        return new FlywheelMotorSubsystem(
            new MotorIOFlywheelStub(IOSettings, flywheelSettings), name, flywheelSettings);
    }
  }

  private ArmMotorSubsystem createArm(Properties robotProperties, String name) {
    ArmSettings armSettings = new ArmSettings();
    String armSettingsPrefix = name + ".armSettings";

    armSettings.color =
        new Color8Bit(
            Integer.parseInt(robotProperties.getProperty(armSettingsPrefix + ".color.red")),
            Integer.parseInt(robotProperties.getProperty(armSettingsPrefix + ".color.green")),
            Integer.parseInt(robotProperties.getProperty(armSettingsPrefix + ".color.blue")));

    armSettings.minAngleInDegrees =
        Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".minAngleInDegrees"));
    armSettings.maxAngleInDegrees =
        Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".maxAngleInDegrees"));
    armSettings.startingAngleInDegrees =
        Double.parseDouble(
            robotProperties.getProperty(armSettingsPrefix + ".startingAngleInDegrees"));
    armSettings.maxVelocityInDegreesPerSecond =
        Double.parseDouble(
            robotProperties.getProperty(armSettingsPrefix + ".maxVelocityInDegreesPerSecond"));
    armSettings.maxAccelerationInDegreesPerSecondSquared =
        Double.parseDouble(
            robotProperties.getProperty(
                armSettingsPrefix + ".maxAccelerationInDegreesPerSecondSquared"));

    armSettings.feedforward =
        new ArmFeedforward(
            Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".feedforward.ks")),
            Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".feedforward.kg")),
            Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".feedforward.kv")),
            Double.parseDouble(
                robotProperties.getProperty(armSettingsPrefix + ".feedforward.ka", "0.0")));

    armSettings.motor = getDCMotor(robotProperties.getProperty(armSettingsPrefix + ".DCMotor"));
    armSettings.simulateGravity =
        Boolean.parseBoolean(robotProperties.getProperty(armSettingsPrefix + ".simulateGravity"));
    armSettings.armLengthInMeters =
        Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".armLengthInMeters"));
    armSettings.armMassInKg =
        Double.parseDouble(robotProperties.getProperty(armSettingsPrefix + ".armMassInKg"));

    MotorIOBaseSettings IOSettings = getMotorIOBaseSettings(robotProperties, name);

    switch (robotProperties.getProperty(name + ".motor.motorController")) {
      case "talonFX":
        TalonFxSettings talonSettings = TalonFxSettings.getSettings(robotProperties, name);
        return new ArmMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, armSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = SparkMaxSettings.getSettings(robotProperties, name);
        return new ArmMotorSubsystem(
            new MotorIOSparkMax(IOSettings, sparkMaxSettings), name, armSettings);

      case "sim":
      default:
        return new ArmMotorSubsystem(
            new MotorIOArmStub(IOSettings, armSettings), name, armSettings);
    }
  }

  private ElevatorMotorSubsystem createElevator(Properties robotProperties, String name) {
    ElevatorSettings elevatorSettings = new ElevatorSettings();
    String elevatorSettingsPrefix = name + ".elevatorSettings";

    elevatorSettings.color =
        new Color8Bit(
            Integer.parseInt(robotProperties.getProperty(elevatorSettingsPrefix + ".color.red")),
            Integer.parseInt(robotProperties.getProperty(elevatorSettingsPrefix + ".color.green")),
            Integer.parseInt(robotProperties.getProperty(elevatorSettingsPrefix + ".color.blue")));

    elevatorSettings.minHeightInMeters =
        Double.parseDouble(
            robotProperties.getProperty(elevatorSettingsPrefix + ".minHeightInMeters"));
    elevatorSettings.maxHeightInMeters =
        Double.parseDouble(
            robotProperties.getProperty(elevatorSettingsPrefix + ".maxHeightInMeters"));
    elevatorSettings.startingHeightInMeters =
        Double.parseDouble(
            robotProperties.getProperty(elevatorSettingsPrefix + ".startingHeightInMeters"));
    elevatorSettings.maxVelocityInMetersPerSecond =
        Double.parseDouble(
            robotProperties.getProperty(elevatorSettingsPrefix + ".maxVelocityInMetersPerSecond"));
    elevatorSettings.maxAccelerationInMetersPerSecondSquared =
        Double.parseDouble(
            robotProperties.getProperty(
                elevatorSettingsPrefix + ".maxAccelerationInMetersPerSecondSquared"));

    elevatorSettings.feedforward =
        new ElevatorFeedforward(
            Double.parseDouble(
                robotProperties.getProperty(elevatorSettingsPrefix + ".feedforward.ks")),
            Double.parseDouble(
                robotProperties.getProperty(elevatorSettingsPrefix + ".feedforward.kg")),
            Double.parseDouble(
                robotProperties.getProperty(elevatorSettingsPrefix + ".feedforward.kv")),
            Double.parseDouble(
                robotProperties.getProperty(elevatorSettingsPrefix + ".feedforward.ka", "0.0")));

    elevatorSettings.motor =
        getDCMotor(robotProperties.getProperty(elevatorSettingsPrefix + ".DCMotor"));
    elevatorSettings.simulateGravity =
        Boolean.parseBoolean(
            robotProperties.getProperty(elevatorSettingsPrefix + ".simulateGravity"));
    elevatorSettings.carriageMassKg =
        Double.parseDouble(robotProperties.getProperty(elevatorSettingsPrefix + ".carriageMassKg"));

    MotorIOBaseSettings IOSettings = getMotorIOBaseSettings(robotProperties, name);

    switch (robotProperties.getProperty(name + ".motor.motorController")) {
      case "talonFX":
        TalonFxSettings talonSettings = TalonFxSettings.getSettings(robotProperties, name);
        return new ElevatorMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, elevatorSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = SparkMaxSettings.getSettings(robotProperties, name);
        return new ElevatorMotorSubsystem(
            new MotorIOSparkMax(IOSettings, sparkMaxSettings), name, elevatorSettings);

      case "sim":
      default:
        return new ElevatorMotorSubsystem(
            new MotorIOElevatorStub(IOSettings, elevatorSettings), name, elevatorSettings);
    }
  }

  private MotorIOBaseSettings getMotorIOBaseSettings(Properties robotProperties, String name) {
    MotorIOBaseSettings IOSettings = new MotorIOBaseSettings();
    String IOSettingsPrefix = name + ".IOSettings";

    IOSettings.motor.inverted =
        Boolean.parseBoolean(robotProperties.getProperty(IOSettingsPrefix + ".inverted"));
    IOSettings.motor.gearing =
        Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".gearing"));
    IOSettings.motor.drumRadiusMeters =
        Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".drumRadiusMeters"));

    IOSettings.pid =
        new PIDController(
            Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".pid.kp")),
            Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".pid.ki")),
            Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".pid.kd")));

    return IOSettings;
  }

  private DCMotor getDCMotor(String motor) {
    switch (motor) {
      case "KrakenX60":
        return DCMotor.getKrakenX60(1);
      case "Neo550":
        return DCMotor.getNeo550(1);
      // more motors if needed
      default:
        return DCMotor.getKrakenX60(1);
    }
  }

  private VisionSubsystem createVisionSubsystem(Properties robotProperties) {
    VisionSettings settings = VisionSettings.getSettings(robotProperties);

    if (Boolean.parseBoolean(robotProperties.getProperty("vision.addDrivetrain"))) {
      vision =
          new VisionSubsystem(
              AprilTagFieldLayout.loadField(
                  AprilTagFields.valueOf(robotProperties.getProperty("vision.fieldlayout"))),
              Optional.of(drive::addVisionMeasurement),
              settings);
    } else {
      vision =
          new VisionSubsystem(
              AprilTagFieldLayout.loadField(
                  AprilTagFields.valueOf(robotProperties.getProperty("vision.fieldlayout"))),
              Optional.empty(),
              settings);
    }

    String[] cameraNames = robotProperties.getProperty("vision.cameras", "").split(", ");
    for (String cameraName : cameraNames) {
      switch (robotProperties.getProperty(cameraName + ".cameraType")) {
        case "photon":
          vision.addCamera(
              CameraPhoton.createCameraPhoton(
                  robotProperties, cameraName, vision.getFieldLayout()));
          break;
        case "photonSim":
          vision.addCamera(
              CameraPhotonSim.createCameraPhotonSim(
                  robotProperties, cameraName, vision.getFieldLayout(), () -> drive.getPose()));
          break;
        default:
          vision.addCamera(CameraBase.createCameraBase(robotProperties, cameraName));
          break;
      }
    }
    return vision;
  }
}
