package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Motor;
import frc.robot.subsystems.interfaces.SimpleMotor;

public class ArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // Y-button = up arm
  // A-button = down arm
  public static void setupController(SimpleMotor arm, CommandXboxController controller) {

    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) arm,
            () -> {
              if (controller.y().getAsBoolean()) {
                return 0.2;
              } else if (controller.a().getAsBoolean()) {
                return -0.2;
              }
              return 0.0;
            }));
  }
}
