package frc.robot.commands.common.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {
  Elevator elevator;
  DoubleSupplier speed;
  double targetPosition;

  /**
   * Sets a elevator to it's current position on meters and incremnts by a speed * the elevator max
   * velocity / 50 each cycle. This command does not end without being interupted.
   *
   * @param elevator
   * @param speed A double between -1 and 1
   */
  public ElevatorCommand(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {
    targetPosition = elevator.getCurrentHeight();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    if (0 != currentSpeed) {
      targetPosition += currentSpeed * elevator.getSettings().maxVelocityInMetersPerSecond / 50;
      targetPosition =
          MathUtil.clamp(
              targetPosition,
              elevator.getSettings().minHeightInMeters,
              elevator.getSettings().maxHeightInMeters);

      elevator.setTargetHeight(targetPosition);
    }
  }
}
