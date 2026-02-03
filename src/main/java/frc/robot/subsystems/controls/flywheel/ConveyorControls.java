package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class ConveyorControls {
  public static void setupController(
      Flywheel conveyorFlywheel, CommandXboxController controller) {

    SubsystemBase conveyorFlywheelSubsystem = (SubsystemBase) conveyorFlywheel;
    conveyorFlywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) conveyorFlywheel,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(conveyorFlywheelSubsystem.getName())) {
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
        .b()
        .onTrue(
            new InstantCommand(() -> ((Motor) conveyorFlywheel).runVoltage(0.0), conveyorFlywheelSubsystem));
}}