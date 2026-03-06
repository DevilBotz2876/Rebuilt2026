package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.flywheel.FlywheelCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

// set the default commands for the intake subsystems
public class IntakeControls {
  public static void setupVoltageController(
      Flywheel intakeFlywheel, CommandXboxController controller) {

    /*
     * Top Flywheel
     * Right Trigger: Stop(0 volts)
     * When top flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase intakeFlywheelSubsystem = (SubsystemBase) intakeFlywheel;
    intakeFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) intakeFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(intakeFlywheelSubsystem.getName())) {
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
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> ((Motor) intakeFlywheel).runVoltage(0.0), intakeFlywheelSubsystem));
  }

  public static void setupSpeedController(
      Flywheel intakeFlywheel, CommandXboxController controller) {

    /*
     * Top Flywheel
     * Right Trigger: Stop(0 volts)
     * When top flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase intakeFlywheelSubsystem = (SubsystemBase) intakeFlywheel;
    intakeFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            intakeFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(intakeFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller.rightTrigger().onTrue(new FlywheelToVelocity(intakeFlywheel, () -> 0.0));
  }

  public static void setupMainController(
      Flywheel intakeFlywheel, CommandXboxController controller) {
    SubsystemBase intakeFlywheelSubsystem = (SubsystemBase) intakeFlywheel;
    intakeFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            intakeFlywheel,
            () -> {
              return 0.0;
            }));
  }
}
