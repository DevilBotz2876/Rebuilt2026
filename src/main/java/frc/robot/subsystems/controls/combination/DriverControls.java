package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorPitCommand;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class DriverControls {
  public static class DriverControlsSettings {
    // launch
    public static double shooterLaunchRPM = 3666.0;
    public static double indexerLaunchRPM = 1000.0;
    public static double conveyorLaunchRPM = 1000.0;

    // intake
    public static double intakeRPM = 2000.0;
    public static double conveyorReverseRPM = -200.0;
  }

  public static void setupMainController(
      Drive drive,
      Flywheel topIntake,
      Flywheel bottomIntake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      CommandXboxController controller) {

    // Commands to stop subsystem
    Command stopShooter = new MotorRunVoltageCommand((Motor) shooter, () -> 0.0);
    Command stopIndexer = new MotorRunVoltageCommand((Motor) indexer, () -> 0.0);
    Command stopConveyor = new MotorRunVoltageCommand((Motor) conveyor, () -> 0.0);
    Command stopTopIntake = new MotorRunVoltageCommand((Motor) topIntake, () -> 0.0);
    Command stopBottomIntake = new MotorRunVoltageCommand((Motor) bottomIntake, () -> 0.0);

    // Launching related commands
    Command launchSequentialParallel =
        new SequentialCommandGroup(
            new FlywheelToVelocity(shooter, () -> DriverControlsSettings.shooterLaunchRPM),
            new ParallelCommandGroup(
                new FlywheelToVelocity(indexer, () -> DriverControlsSettings.indexerLaunchRPM),
                new FlywheelToVelocity(conveyor, () -> DriverControlsSettings.conveyorLaunchRPM)));

    Command launchAllSequential =
        new SequentialCommandGroup(
            new FlywheelToVelocity(shooter, () -> DriverControlsSettings.shooterLaunchRPM),
            new FlywheelToVelocity(indexer, () -> DriverControlsSettings.indexerLaunchRPM),
            new FlywheelToVelocity(conveyor, () -> DriverControlsSettings.conveyorLaunchRPM));

    Command stopLaunch = stopShooter.alongWith(stopIndexer, stopConveyor);

    // Intake Commands

    Command intakeIn =
        new ParallelCommandGroup(
            new FlywheelToVelocity(topIntake, () -> DriverControlsSettings.intakeRPM),
            new FlywheelToVelocity(bottomIntake, () -> DriverControlsSettings.intakeRPM));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(topIntake, () -> -DriverControlsSettings.intakeRPM),
            new FlywheelToVelocity(bottomIntake, () -> -DriverControlsSettings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> DriverControlsSettings.conveyorReverseRPM));

    Command stopIntake = stopTopIntake.alongWith(stopBottomIntake);

    // binding
    controller.x().whileTrue(launchSequentialParallel).onFalse(stopLaunch);
    controller.y().whileTrue(launchAllSequential).onFalse(stopLaunch);

    controller.a().whileTrue(intakeIn).onFalse(stopIntake);
    controller.b().whileTrue(intakeOut).onFalse(stopIntake);
    controller.b().onFalse(stopConveyor);
  }

  public static void setupAssistController(
      Drive drive,
      Flywheel topIntake,
      Flywheel bottomIntake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      CommandXboxController controller) {}

  public static void setupFlywheelSmartDashboardControl(Flywheel flywheel) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) flywheel;
    setupMotorSmartDashboardControl(flywheelSubsystem, (Motor) flywheel);
    setupSmartDashboardVoltageControl(flywheelSubsystem, (Motor) flywheel);
    setupSmartDashboardSpeedControl(flywheelSubsystem, flywheel);
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
