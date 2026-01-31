package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class ShooterControls {
  public static void setupVoltageController(
      Flywheel shooterFlywheel, Flywheel indexerFlywheel, CommandXboxController controller) {

    /*
     * Top Flywheel
     * Right Trigger: Stop(0 volts)
     * When top flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase shooterFlywheelSubsystem = (SubsystemBase) shooterFlywheel;
    shooterFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) shooterFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystem", "UNKNOWN")
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
            new InstantCommand(() -> ((Motor) shooterFlywheel).runVoltage(0.0), shooterFlywheelSubsystem));

    /*
     * Bottom Flywheel
     * Right Bumper: Stop (0 volts)
     * When bottom flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase indexerFlywheelSubsystem = (SubsystemBase) indexerFlywheel;
    indexerFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) indexerFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystem", "UNKNOWN")
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
            new InstantCommand(
                () -> ((Motor) indexerFlywheel).runVoltage(0.0), indexerFlywheelSubsystem));
  }

  public static void setupSmartDashboardControl(Flywheel flywheel) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) flywheel;
    SmartDashboard.putString("Selected Subsystem", "UNKNOWN");
    SmartDashboard.putData(
        "Select " + flywheelSubsystem.getName(),
        new InstantCommand(
            () -> SmartDashboard.putString("Selected Subsystem", flywheelSubsystem.getName())));
  }
}
