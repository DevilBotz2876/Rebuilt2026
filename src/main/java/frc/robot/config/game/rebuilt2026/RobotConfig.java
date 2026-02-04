package frc.robot.config.game.rebuilt2026;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
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
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.controls.flywheel.ConveyorControls;
import frc.robot.subsystems.controls.flywheel.IntakeControls;
import frc.robot.subsystems.controls.flywheel.ShooterControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveCTRE;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.implementations.motor.SimpleMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;
import frc.robot.subsystems.interfaces.SimpleMotor.SimpleMotorSettings;
import java.util.Properties;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public DriveBase drive;
  public SendableChooser<Command> autoChooser;
  public FlywheelMotorSubsystem topIntakeFlywheel;
  public FlywheelMotorSubsystem bottomIntakeFlywheel;
  public FlywheelMotorSubsystem shooterFlywheel;
  public FlywheelMotorSubsystem indexerFlywheel;

  public FlywheelMotorSubsystem conveyorFlywheel;
  // TODO: Add VisionSubsystem Declaration

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(Properties robotProperties) {
    if (robotProperties.containsKey("robot.drive")) {
      if (robotProperties.getProperty("robot.drive").equals("ctre")) {
        drive = new DriveSwerveCTRE(new TunerConstants(robotProperties));
      }
    } else {
      drive = new DriveBase("Stub");
    }

    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    }

    topIntakeFlywheel = createFlywheel(robotProperties, "topIntakeFlywheel");
    bottomIntakeFlywheel = createFlywheel(robotProperties, "bottomIntakeFlywheel");
    shooterFlywheel = createFlywheel(robotProperties, "shooterFlywheel");
    indexerFlywheel = createFlywheel(robotProperties, "indexerFlywheel");
    conveyorFlywheel = createFlywheel(robotProperties, "conveyorFlywheel");
  }

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    if (stubDrive) {
      drive = new DriveBase("Stub");
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    // TODO: Add VisionSubsystem Initialization

    if (stubVision) {
      // TODO: Add VisionSubsystem Settings
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      // TODO: Add VisionSubsystem Simulation Support

      // HACK just to verify autos are visible without connecting to robot
      // this.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }
    DriveControls.setupController(drive, mainController);

    // Send vision-based odometry measurements to drive's odometry calculations
    // vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);
    IntakeControls.setupVoltageController(
        topIntakeFlywheel,
        bottomIntakeFlywheel,
        mainController); // .setupVoltageController(topIntakeFlywheel, bottomIntakeFlywheel,
    // mainController);
    IntakeControls.setupSmartDashboardControl(topIntakeFlywheel);
    IntakeControls.setupSmartDashboardControl(bottomIntakeFlywheel);
    ShooterControls.setupVoltageController(shooterFlywheel, indexerFlywheel, mainController);
    ShooterControls.setupSmartDashboardControl(shooterFlywheel);
    ShooterControls.setupSmartDashboardControl(indexerFlywheel);
    ConveyorControls.setupController(conveyorFlywheel, mainController);
    ShooterControls.setupSmartDashboardControl(conveyorFlywheel);

    if (null != this.autoChooser) {
      SmartDashboard.putData("Autonomous", this.autoChooser);
    }
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
        TalonFxSettings talonSettings = new TalonFxSettings();
        talonSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".talonFX.setting.id"));
        return new SimpleMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, simpleMotorSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
        sparkMaxSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".sparkMax.setting.id"));
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
        TalonFxSettings talonSettings = new TalonFxSettings();
        talonSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".talonFX.setting.id"));
        return new FlywheelMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, flywheelSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
        sparkMaxSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".sparkMax.setting.id"));
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
        TalonFxSettings talonSettings = new TalonFxSettings();
        talonSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".talonFX.setting.id"));
        return new ArmMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, armSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
        sparkMaxSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".sparkMax.setting.id"));
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
        TalonFxSettings talonSettings = new TalonFxSettings();
        talonSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".talonFX.setting.id"));
        return new ElevatorMotorSubsystem(
            new MotorIOTalonFx(IOSettings, talonSettings), name, elevatorSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
        sparkMaxSettings.canId =
            Integer.parseInt(robotProperties.getProperty(name + ".sparkMax.setting.id"));
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
            Double.parseDouble(robotProperties.getProperty(IOSettingsPrefix + ".pid.kp")));

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
}
