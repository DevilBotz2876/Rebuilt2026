package frc.robot.subsystems.controls.combination;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import java.util.Optional;
import java.util.Properties;

public class DriverControls {
  public static class DriverControlsSettings {
    // launch
    public double shooterCurrentLaunchRPM;
    public double shooterOutpostLaunchRPM;
    public double shooterTrenchLaunchRPM;
    public double shooterDepotLaunchRPM;
    public double shooterAgainstHubLaunchRPM;
    public double shooterPassRPM;
    public double indexerLaunchRPM;
    public double conveyorLaunchRPM;
    public double indexerOutpostLaunchRPM;
    public double conveyorOutpostLaunchRPM;

    // intake
    public double intakeRPM;
    public double intakeReverseRPM;
    public double conveyorReverseRPM;
    public double intakeArmVolts;
    public double intakeArmWhileInakingVolts;
    public double intakeDriveSpeed;

    /**
     * Creates a DriverControlsSettings and sets the varibles to equal the matching property value.
     *
     * @param properties The robot's proerties
     * @return DriverControlsSettings with values matching the Properties
     */
    public static DriverControlsSettings getDriverControlsSettings(Properties properties) {
      DriverControlsSettings settings = new DriverControlsSettings();
      settings.shooterCurrentLaunchRPM =
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
      settings.intakeArmVolts =
          Double.parseDouble(properties.getProperty("driverControls.intakeArmVolts"));
      settings.indexerOutpostLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.indexerOutpostLaunchRPM"));
      settings.conveyorOutpostLaunchRPM =
          Double.parseDouble(properties.getProperty("driverControls.conveyorOutpostLaunchRPM"));
      settings.intakeDriveSpeed =
          Double.parseDouble(properties.getProperty("driverControls.intakeDriveSpeed"));
      settings.intakeArmWhileInakingVolts =
          Double.parseDouble(properties.getProperty("driverControls.intakeArmWhileInakingVolts"));
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
    SmartDashboard.putNumber("Controls/launchShooterRPM", settings.shooterCurrentLaunchRPM);
    SmartDashboard.putBoolean("Controls/isShooting", false);

    Command launchSequentialParallelSmartDashBoard =
        new SequentialCommandGroup(
            new FlywheelToVelocity(
                shooter,
                () ->
                    SmartDashboard.getNumber(
                        "Controls/launchShooterRPM", settings.shooterCurrentLaunchRPM)),
            new WaitCommand(0.2),
            new ParallelCommandGroup(
                new FlywheelToVelocity(
                    indexer,
                    () -> {
                      if (SmartDashboard.getNumber(
                              "Controls/launchShooterRPM", settings.shooterCurrentLaunchRPM)
                          == settings.shooterOutpostLaunchRPM) {
                        return settings.indexerOutpostLaunchRPM;
                      }
                      return settings.indexerLaunchRPM;
                    }),
                new FlywheelToVelocity(
                    conveyor,
                    () -> {
                      if (SmartDashboard.getNumber(
                              "Controls/launchShooterRPM", settings.shooterCurrentLaunchRPM)
                          == settings.shooterOutpostLaunchRPM) {
                        return settings.conveyorOutpostLaunchRPM;
                      }
                      return settings.conveyorLaunchRPM;
                    })));

    // Intake Commands
    Command intakeIn =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> settings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> settings.conveyorLaunchRPM),
            new MotorRunVoltageCommand(
                (Motor) intakeArm, () -> settings.intakeArmWhileInakingVolts));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> settings.intakeReverseRPM),
            new FlywheelToVelocity(conveyor, () -> settings.conveyorReverseRPM));

    Command DeployerVoltageMinus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> -settings.intakeArmVolts);
    Command DeployerVoltagePlus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> settings.intakeArmVolts);

    // binding

    // control rumble during shift. Unclear if it works
    // SmartDashboard.putBoolean("Controls/isHubActive", isHubActive());
    // Trigger activeHubChanged = new Trigger(() ->
    // SmartDashboard.getBoolean("Controls/isHubActive", false) != isHubActive());
    // activeHubChanged.onTrue(new InstantCommand(() ->
    // SmartDashboard.putBoolean("Controls/isHubActive", isHubActive()))).onTrue(new
    // SequentialCommandGroup(new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble,
    // 1)), new WaitCommand(2), new InstantCommand(() ->
    // controller.setRumble(RumbleType.kBothRumble, 0))));

    // shooting

    // unclear if this works
    Trigger launchShooterRPMChanged =
        new Trigger(
            () ->
                SmartDashboard.getNumber("Controls/launchShooterRPM", 0)
                    != settings.shooterCurrentLaunchRPM);

    launchShooterRPMChanged.onTrue(
        new InstantCommand(
            () ->
                settings.shooterCurrentLaunchRPM =
                    SmartDashboard.getNumber("Controls/launchShooterRPM", 0)));

    launchShooterRPMChanged
        .and(controller.rightTrigger())
        .onTrue(
            new InstantCommand(
                () -> {
                  launchSequentialParallelSmartDashBoard.cancel();
                  launchSequentialParallelSmartDashBoard.schedule();
                }));

    controller
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Controls/isShooting", true)))
        .whileTrue(launchSequentialParallelSmartDashBoard)
        .onFalse(stopShooter)
        .onFalse(stopIndexer)
        .onFalse(stopConveyor)
        .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Controls/isShooting", false)));

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

    Command stopShooter = new MotorRunVoltageCommand((Motor) shooter, () -> 0.0);
    Command stopIndexer = new MotorRunVoltageCommand((Motor) indexer, () -> 0.0);
    Command stopConveyor = new MotorRunVoltageCommand((Motor) conveyor, () -> 0.0);
    Command stopIntake = new MotorRunVoltageCommand((Motor) intake, () -> 0.0);
    Command stopIntakeArm = new MotorRunVoltageCommand((Motor) intakeArm, () -> 0.0);

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

    Command DeployerVoltageMinus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> -settings.intakeArmVolts);
    Command DeployerVoltagePlus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> settings.intakeArmVolts);

    controller.pov(90).whileTrue(DeployerVoltagePlus).onFalse(stopIntakeArm);
    controller.pov(270).whileTrue(DeployerVoltageMinus).onFalse(stopIntakeArm);

    controller.leftTrigger().onTrue(intakeOut).onFalse(stopIntake).onFalse(stopConveyor);
  }

  public static void setupPitControls() {
    // SmartDashboard.putData("Pit/PDP", new PowerDistribution(1, ModuleType.kRev));
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
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Run at x Volts/Run 0 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 0));
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

  // Taken the game specific message page on the wpilib docs
  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
