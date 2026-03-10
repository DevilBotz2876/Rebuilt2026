package frc.robot.subsystems.controls.combination;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorPitCommand;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;
import java.util.Properties;

public class DriverControls {
  public static class DriverControlsSettings {
    // launch
    public double shooterDefaultLaunchRPM;
    public double shooterOutpostLaunchRPM;
    public double shooterTrenchLaunchRPM;
    public double shooterDepotLaunchRPM;
    public double shooterAgainstHubLaunchRPM;
    public double shooterPassRPM;
    public double indexerLaunchRPM;
    public double conveyorLaunchRPM;

    // intake
    public double intakeRPM;
    public double intakeReverseRPM;
    public double conveyorReverseRPM;
    public double intakeDriveSpeed;

    /**
     * Creates a DriverControlsSettings and sets the varibles to equal the matching property value.
     *
     * @param properties The robot's proerties
     * @return DriverControlsSettings with values matching the Properties
     */
    public static DriverControlsSettings getDriverControlsSettings(Properties properties) {
      DriverControlsSettings settings = new DriverControlsSettings();
      settings.shooterDefaultLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterDefaultLaunchRPM"));
      settings.shooterOutpostLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterOutpostLaunchRPM"));
      settings.shooterTrenchLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterTrenchLaunchRPM"));
      settings.shooterAgainstHubLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterAgainstHubLaunchRPM"));
      settings.shooterDepotLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterDepotLaunchRPM"));
      settings.shooterPassRPM =
          Double.parseDouble(properties.getProperty("driverControls.shooterPassRPM"));

      settings.indexerLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.indexerLaunchRPM"));
      settings.conveyorLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.conveyorLaunchRPM"));

      settings.intakeRPM = Double.parseDouble(properties.getProperty("driverControls.intakeRPM"));
      settings.intakeReverseRPM =
          Double.parseDouble(properties.getProperty("driverControls.intakeReverseRPM"));
      settings.conveyorReverseRPM =
          Double.parseDouble(properties.getProperty("driverControls.conveyorReverseRPM"));
      settings.intakeDriveSpeed = Double.parseDouble(properties.getProperty("driverControls.intakeDriveSpeed"));
      return settings;
    }
  }

  public static void setupMainController(
      Drive drive,
      Flywheel intake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      Arm intakeArm,
      CommandXboxController controller,
      DriverControlsSettings settings) {

    // Commands to stop subsystem
    Command stopShooter = new MotorRunVoltageCommand((Motor) shooter, () -> 0.0);
    Command stopIndexer = new MotorRunVoltageCommand((Motor) indexer, () -> 0.0);
    Command stopConveyor = new MotorRunVoltageCommand((Motor) conveyor, () -> 0.0);
    Command stopIntake = new MotorRunVoltageCommand((Motor) intake, () -> 0.0);
    Command stopIntakeArm = new MotorRunVoltageCommand((Motor) intakeArm, () -> 0.0);

    // Launching related commands
    SmartDashboard.putNumber("Controls/launchShooterRPM", settings.shooterDefaultLaunchRPM);
    Command launchSequentialParallelSmartDashBoard =
        new SequentialCommandGroup(
            new FlywheelToVelocity(
                shooter,
                () ->
                    SmartDashboard.getNumber(
                        "Controls/launchShooterRPM", settings.shooterDefaultLaunchRPM)),
            new ParallelCommandGroup(
                new FlywheelToVelocity(indexer, () -> settings.indexerLaunchRPM),
                new FlywheelToVelocity(conveyor, () -> settings.conveyorLaunchRPM)));

    // Intake Commands
    Command intakeIn =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> settings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> settings.conveyorLaunchRPM),
            new MotorRunVoltageCommand((Motor) intakeArm, () -> -1.0));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> settings.intakeReverseRPM),
            new FlywheelToVelocity(conveyor, () -> settings.conveyorReverseRPM));

    Command DeployerVoltageMinus = new MotorRunVoltageCommand((Motor) intakeArm, () -> -0.5);
    Command DeployerVoltagePlus = new MotorRunVoltageCommand((Motor) intakeArm, () -> 0.5);

    // binding

    // shooting
    controller
        .rightTrigger()
        .whileTrue(launchSequentialParallelSmartDashBoard)
        .onFalse(stopShooter)
        .onFalse(stopIndexer)
        .onFalse(stopConveyor);

    // set shooter speed
    controller
        .y()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterOutpostLaunchRPM)));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterTrenchLaunchRPM)));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterAgainstHubLaunchRPM)));

    // intake
    controller
        .leftTrigger()
        .whileTrue(intakeIn)
        .onFalse(stopIntake)
        .onFalse(stopConveyor)
        .onFalse(stopIntakeArm);
    controller.leftBumper().whileTrue(intakeOut).onFalse(stopIntake).onFalse(stopConveyor);

    // deployer
    controller.pov(90).whileTrue(DeployerVoltagePlus).onFalse(stopIntakeArm);
    controller.pov(270).whileTrue(DeployerVoltageMinus).onFalse(stopIntakeArm);

    SmartDashboard.putNumber("Controls/intakeDriveSpeed", settings.intakeDriveSpeed);
    Trigger intakeDriveSpeedchange =
        new Trigger(
            () ->
                SmartDashboard.getNumber("Controls/intakeDriveSpeed", settings.intakeDriveSpeed)
                    != settings.intakeDriveSpeed);
    intakeDriveSpeedchange.onTrue(
        new InstantCommand(
            () ->
                settings.intakeDriveSpeed =
                    SmartDashboard.getNumber(
                        "Controls/intakeDriveSpeed", settings.intakeDriveSpeed)));
    Command driveForintakeCommand =
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05) * settings.intakeDriveSpeed,
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05) * settings.intakeDriveSpeed,
            () ->
                MathUtil.applyDeadband(-controller.getRightX(), 0.05) * settings.intakeDriveSpeed);
    controller
        .leftTrigger()
        .onTrue(
            driveForintakeCommand.until(() -> controller.leftTrigger().negate().getAsBoolean()));
  }

  public static void setupAssistController(
      Drive drive,
      Flywheel intake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      Arm intakeArm,
      CommandXboxController controller,
      DriverControlsSettings settings) {
    controller
        .y()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterPassRPM)));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterTrenchLaunchRPM)));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterAgainstHubLaunchRPM)));

    controller
        .b()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterDepotLaunchRPM)));
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () ->
                    SmartDashboard.putNumber(
                        "Controls/launchShooterRPM", settings.shooterOutpostLaunchRPM)));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> settings.intakeReverseRPM),
            new FlywheelToVelocity(conveyor, () -> settings.conveyorReverseRPM));

    controller.leftTrigger().onTrue(intakeOut);
  }

  public static void setupFlywheelSmartDashboardControl(Flywheel flywheel) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) flywheel;
    setupMotorSmartDashboardControl(flywheelSubsystem, (Motor) flywheel);
    setupSmartDashboardVoltageControl(flywheelSubsystem, (Motor) flywheel);
    setupSmartDashboardSpeedControl(flywheelSubsystem, flywheel);
  }

  public static void setupArmSmartDashboardControl(Arm arm) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    setupMotorSmartDashboardControl(armSubsystem, (Motor) arm);
    setupSmartDashboardVoltageControl(armSubsystem, (Motor) arm);
  }

  private static void setupMotorSmartDashboardControl(SubsystemBase motorSubsystem, Motor motor) {
    SmartDashboard.putString("Selected Subsystems/Selected", "UNKNOWN");
    SmartDashboard.putData(
        "Selected Subsystems/Select " + motorSubsystem.getName(),
        new InstantCommand(
            () ->
                SmartDashboard.putString(
                    "Selected Subsystems/Selected", motorSubsystem.getName())));
  }

  private static void setupSmartDashboardVoltageControl(SubsystemBase motorSubsystem, Motor motor) {
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run -10.0 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -10.0));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run -7.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -7.5));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run -5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -5.0));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run -2.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -2.5));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run 10.0 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 10.0));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run 7.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 7.5));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run 5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 5.0));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run 2.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 2.5));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at set Voltage",
        new MotorPitCommand((Motor) motor, motorSubsystem.getName() + "/Commands/Set Voltage"));
  }

  private static void setupSmartDashboardSpeedControl(
      SubsystemBase flywheelSubsystem, Flywheel flywheel) {

    SmartDashboard.putNumber(flywheelSubsystem.getName() + "/Commands/Set Speed", 0.0);
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at set RPM",
        new FlywheelToVelocity(
            flywheel,
            () ->
                SmartDashboard.getNumber(
                    flywheelSubsystem.getName() + "/Commands/Set Speed", 0.0)));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -1000 RPM",
        new FlywheelToVelocity(flywheel, () -> -1000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -2000 RPM",
        new FlywheelToVelocity(flywheel, () -> -2000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -4000 RPM",
        new FlywheelToVelocity(flywheel, () -> -4000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -6000 RPM",
        new FlywheelToVelocity(flywheel, () -> -6000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 0 RPM",
        new FlywheelToVelocity(flywheel, () -> 0.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 1000 RPM",
        new FlywheelToVelocity(flywheel, () -> 1000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 2000 RPM",
        new FlywheelToVelocity(flywheel, () -> 2000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 4000 RPM",
        new FlywheelToVelocity(flywheel, () -> 4000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 6000 RPM",
        new FlywheelToVelocity(flywheel, () -> 6000.0));
  }
}
