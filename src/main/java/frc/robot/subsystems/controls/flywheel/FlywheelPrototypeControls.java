package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.flywheel.FlywheelCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorPitCommand;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class FlywheelPrototypeControls {
  static int returnFactor;

  public static void setupController(Flywheel motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;

    motorSubsystem.setDefaultCommand(
        new MotorRunVoltageCommand(
            (Motor) motor,
            () -> {
              if (controller.rightBumper().getAsBoolean()) {
                returnFactor = -1;
              } else {
                returnFactor = 1;
              }
              if (controller.a().getAsBoolean()) {
                return 2.5 * returnFactor;
              } else if (controller.x().getAsBoolean()) {
                return 5.0 * returnFactor;
              } else if (controller.y().getAsBoolean()) {
                return 7.5 * returnFactor;
              } else if (controller.b().getAsBoolean()) {
                return 10.0 * returnFactor;
              }
              return 0.0;
            }));
  }

  public static void setupController2(Flywheel motor, CommandXboxController controller) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) motor;
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Select motor",
        new InstantCommand(
            () ->
                SmartDashboard.putString(
                    "Prototype/Selected Motor/", flywheelSubsystem.getName())));
    Command c =
        new ConditionalCommand(
            new MotorRunVoltageCommand((Motor) motor, () -> controller.getLeftY()),
            new MotorRunVoltageCommand((Motor) motor, () -> 0.0),
            () ->
                SmartDashboard.getString("Prototype/Selected Motor/", "NULL")
                    == flywheelSubsystem.getName());
    flywheelSubsystem.setDefaultCommand(c);
  }

  public static void setupControllerTwo(Flywheel motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;

    motorSubsystem.setDefaultCommand(
        new MotorRunVoltageCommand(
            (Motor) motor,
            () -> {
              if (controller.leftBumper().getAsBoolean()) {
                returnFactor = -1;
              } else {
                returnFactor = 1;
              }
              if (controller.pov(180).getAsBoolean()) {
                return 2.5 * returnFactor;
              } else if (controller.pov(270).getAsBoolean()) {
                return 5.0 * returnFactor;
              } else if (controller.pov(0).getAsBoolean()) {
                return 7.5 * returnFactor;
              } else if (controller.pov(90).getAsBoolean()) {
                return 10.0 * returnFactor;
              }
              return 0.0;
            }));
  }

  public static void setupVelocityController(Flywheel motor, CommandXboxController controller) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) motor;
    flywheelSubsystem.setDefaultCommand(
        new FlywheelCommand(
            motor,
            () -> {
              return 0.0;
            }));
  }

  public static void setupSmartDashboardControl(Flywheel motor) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) motor;

    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -10.0 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -10.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -7.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -7.5));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -5.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run -2.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> -2.5));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 10.0 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 10.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 7.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 7.5));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 5.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x Volts/Run 2.5 Volts",
        new MotorRunVoltageCommand((Motor) motor, () -> 2.5));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at set Voltage",
        new MotorPitCommand((Motor) motor, flywheelSubsystem.getName() + "/Commands/Set Voltage"));

    SmartDashboard.putNumber(flywheelSubsystem.getName() + "/Commands/Set Speed", 0.0);
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at set RPM",
        new FlywheelToVelocity(
            motor,
            () ->
                SmartDashboard.getNumber(
                    flywheelSubsystem.getName() + "/Commands/Set Speed", 0.0)));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -1000 RPM",
        new FlywheelToVelocity(motor, () -> -1000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -2000 RPM",
        new FlywheelToVelocity(motor, () -> -2000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -4000 RPM",
        new FlywheelToVelocity(motor, () -> -4000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run -6000 RPM",
        new FlywheelToVelocity(motor, () -> -6000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 0 RPM",
        new FlywheelToVelocity(motor, () -> 0.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 1000 RPM",
        new FlywheelToVelocity(motor, () -> 1000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 2000 RPM",
        new FlywheelToVelocity(motor, () -> 2000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 4000 RPM",
        new FlywheelToVelocity(motor, () -> 4000.0));
    SmartDashboard.putData(
        flywheelSubsystem.getName() + "/Commands/Run at x RPMs/Run 6000 RPM",
        new FlywheelToVelocity(motor, () -> 6000.0));
  }
}
