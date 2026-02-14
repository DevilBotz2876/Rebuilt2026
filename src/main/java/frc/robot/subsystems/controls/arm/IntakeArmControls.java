package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.subsystems.interfaces.Arm;

public class IntakeArmControls {
  // Right Bumper = Increase Rotation CW
  // Left Bumper = Increase Rotation CCW
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        // new MotorBringUpCommand(
        //     (Motor) arm,
        new ArmCommand(
            (Arm) arm,
            () -> {
              if (!SmartDashboard.getString("Selected Subsystems/Selected", "UNKNOWN")
                  .equals(armSubsystem.getName())) return 0.0;
              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To -90", new ArmToPosition((Arm) arm, () -> -90));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To -45", new ArmToPosition((Arm) arm, () -> -45));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 0", new ArmToPosition((Arm) arm, () -> 0));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 15", new ArmToPosition((Arm) arm, () -> 15));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 45", new ArmToPosition((Arm) arm, () -> 45));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Arm To 75", new ArmToPosition((Arm) arm, () -> 75));
  }
}
