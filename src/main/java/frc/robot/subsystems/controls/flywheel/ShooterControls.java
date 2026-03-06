package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.flywheel.FlywheelCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

// set the default commands for the shooter subsystems
public class ShooterControls {
  public static void setupVoltageController(
      Flywheel shooter, Flywheel indexer, CommandXboxController controller) {

    /*
     * Shooter
     * Left Trigger: Stop(0 volts)
     * When Shooter is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase shooterFlywheelSubsystem = (SubsystemBase) shooter;
    shooterFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) shooter,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(shooterFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(() -> ((Motor) shooter).runVoltage(0.0), shooterFlywheelSubsystem));

    /*
     * Indexer
     * Left Bumper: Stop (0 volts)
     * When Indexer is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase indexerFlywheelSubsystem = (SubsystemBase) indexer;
    indexerFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) indexer,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(indexerFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(() -> ((Motor) indexer).runVoltage(0.0), indexerFlywheelSubsystem));
  }

  public static void setupSpeedController(
      Flywheel shooter, Flywheel indexer, CommandXboxController controller) {

    /*
     * Shooter
     * Left Trigger: Stop(0 volts)
     * When Shooter is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase shooterFlywheelSubsystem = (SubsystemBase) shooter;
    shooterFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            shooter,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(shooterFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller.leftTrigger().onTrue(new FlywheelToVelocity(shooter, () -> 0.0));

    /*
     * Indexer
     * Left Bumper: Stop (0 volts)
     * When Indexer is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase indexerFlywheelSubsystem = (SubsystemBase) indexer;
    indexerFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            indexer,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(indexerFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller.leftBumper().onTrue(new FlywheelToVelocity(indexer, () -> 0.0));
  }

  public static void setupMainController(
      Flywheel shooter, Flywheel indexer, CommandXboxController controller) {
    SubsystemBase shooterFlywheelSubsystem = (SubsystemBase) shooter;
    shooterFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            shooter,
            () -> {
              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    SubsystemBase indexerFlywheelSubsystem = (SubsystemBase) indexer;
    indexerFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            indexer,
            () -> {
              return 0.0;
            }));
  }
}
