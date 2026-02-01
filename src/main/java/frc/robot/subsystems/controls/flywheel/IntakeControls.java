package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class IntakeControls {
  public static void setupVoltageController(
      Flywheel topFlywheel, Flywheel bottomFlywheel, CommandXboxController controller) {

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

    /*
     * Bottom Flywheel
     * Right Bumper: Stop (0 volts)
     * When bottom flywheel is selected
     *  Up D-PAD: increase volts
     *  Down D-PAD: decrease volts
     */
    SubsystemBase bottomFlywheelSubsystem = (SubsystemBase) bottomFlywheel;
    bottomFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) bottomFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(bottomFlywheelSubsystem.getName())) {
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
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> ((Motor) bottomFlywheel).runVoltage(0.0), bottomFlywheelSubsystem));
  }

  public static void setupSmartDashboardControl(Flywheel flywheel) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) flywheel;
    SmartDashboard.putString("Selected Subsystems/Selected", "UNKNOWN");
    SmartDashboard.putData(
        "Selected Subsystems/Select " + flywheelSubsystem.getName(),
        new InstantCommand(
            () ->
                SmartDashboard.putString(
                    "Selected Subsystems/Selected", flywheelSubsystem.getName())));
  }
}
