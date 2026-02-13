package frc.robot.commands.common.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.DoubleSupplier;

public class FlywheelCommand extends Command {
  Flywheel flywheel;
  DoubleSupplier speed;
  double targetVelocity;

  /**
   * Sets a flywheel to it's current RPM and incremnts by a speed * the flywheel max velocity / 50
   * each cycle. This command does not end without being interupted.
   *
   * @param flywheel
   * @param speed A double between -1 and 1
   */
  public FlywheelCommand(Flywheel flywheel, DoubleSupplier speed) {
    this.flywheel = flywheel;
    this.speed = speed;

    addRequirements((Subsystem) flywheel);
  }

  @Override
  public void initialize() {
    targetVelocity = flywheel.getCurrentVelocity();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetVelocity += currentSpeed * flywheel.getSettings().maxVelocityInRPMs / 50;
    targetVelocity =
        MathUtil.clamp(
            targetVelocity,
            -flywheel.getSettings().maxVelocityInRPMs,
            flywheel.getSettings().maxVelocityInRPMs);

    flywheel.setTargetVelocity(targetVelocity);
    if (targetVelocity == 0.0) {
      ((Motor) flywheel).runVoltage(0.0);
      return;
    }
  }
}
