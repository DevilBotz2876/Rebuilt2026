package frc.robot.commands.common.motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.DoubleSupplier;

public class MotorRunVoltageCommand extends Command {
  Motor motor;
  DoubleSupplier voltage;

  public MotorRunVoltageCommand(Motor motor, DoubleSupplier voltage) {
    this.motor = motor;
    this.voltage = voltage;

    addRequirements((Subsystem) motor);
  }

  @Override
  public void execute() {
    motor.runVoltage(voltage.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    motor.runVoltage(0.0);
  }
}
