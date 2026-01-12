package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.commands.common.motor.MotorPitCommand;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.implementations.motor.MotorSubsystem;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class FlywheelPrototypeControls {
    public static void setupController(Flywheel motor, CommandXboxController controller) {}

    public static void setupSmartDashboardControl(Flywheel motor) {
        SubsystemBase flywheelSubsystem = (SubsystemBase) motor;

        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -10.0 Volts", new MotorRunVoltageCommand((Motor) motor,  -10.0));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -7.5 Volts", new MotorRunVoltageCommand((Motor) motor,  -7.5));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -5 Volts", new MotorRunVoltageCommand((Motor) motor,  -5.0));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -2.5 Volts", new MotorRunVoltageCommand((Motor) motor,  -2.5));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 10.0 Volts", new MotorRunVoltageCommand((Motor) motor,  10.0));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 7.5 Volts", new MotorRunVoltageCommand((Motor) motor,  7.5));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 5 Volts", new MotorRunVoltageCommand((Motor) motor,  5.0));
        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 2.5 Volts", new MotorRunVoltageCommand((Motor) motor,  2.5));

        SmartDashboard.putData(flywheelSubsystem.getName() + "/Commands/Run at set Voltage", new MotorPitCommand((Motor) motor, flywheelSubsystem.getName() + "/Commands/Set Voltage"));
    }
}
