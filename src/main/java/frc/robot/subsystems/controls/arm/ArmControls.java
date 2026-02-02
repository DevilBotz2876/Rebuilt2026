package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Motor;

public class ArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // Y-button = up arm
  // A-button = down arm
  public static void setupController(Arm arm, CommandXboxController controller) {

    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) arm,
            // new ArmCommand(arm,
            () -> {
              if (controller.y().getAsBoolean()) {
                return 0.2;
              } else if (controller.a().getAsBoolean()) {
                return -0.2;
              }
              return 0.0;
            }));
    // new ArmCommand(
    //     arm,
    //     () -> {
    //       if (controller.povRight().getAsBoolean()) {
    //         return 0.1;
    //       } else if (controller.povLeft().getAsBoolean()) {
    //         return -0.1;
    //       }
    //       return 0.0;
    //     }));

    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To -90", new ArmToPosition(arm, () -> -90));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To -45", new ArmToPosition(arm, () -> -45));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 0", new ArmToPosition(arm, () -> 0));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 15", new ArmToPosition(arm, () -> 15));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 45", new ArmToPosition(arm, () -> 45));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 75", new ArmToPosition(arm, () -> 75));
  }
}
