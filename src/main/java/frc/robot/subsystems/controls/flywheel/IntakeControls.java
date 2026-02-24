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
      Flywheel topFlywheel, CommandXboxController controller) {

    /*
     * Top Flywheel
     * Right Trigger: Stop(0 volts)
     * When top flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase topFlywheelSubsystem = (SubsystemBase) topFlywheel;
    topFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) topFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(topFlywheelSubsystem.getName())) {
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
            new InstantCommand(() -> ((Motor) topFlywheel).runVoltage(0.0), topFlywheelSubsystem));
  }

  public static void setupSpeedController(Flywheel topFlywheel, CommandXboxController controller) {

    /*
     * Top Flywheel
     * Right Trigger: Stop(0 volts)
     * When top flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase topFlywheelSubsystem = (SubsystemBase) topFlywheel;
    topFlywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            topFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(topFlywheelSubsystem.getName())) {
                return 0.0;
              }

              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    controller.rightTrigger().onTrue(new FlywheelToVelocity(topFlywheel, () -> 0.0));
  }
}
