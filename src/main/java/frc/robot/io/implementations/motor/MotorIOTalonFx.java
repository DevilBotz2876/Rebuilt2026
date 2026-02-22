package frc.robot.io.implementations.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import java.util.Properties;

public class MotorIOTalonFx extends MotorIOBase {
  public static class TalonFxSettings {
    public int canId = 0;
    public double supplyCurrentLimitAmps = 70.0;
    public double supplyCurrentLowerLimitAmps = 40.0;
    public double supplyCurrentLowerTimeSeconds = 1.0;

    public double statorCurrentLimitAmps = 120.0;
    public boolean enableStatorCurrentLimit;

    public double peakForwardVoltage = 12.0;
    public double peakReverseVoltage = -12.0;

    /**
     * Created and returns the TalonFxSettings from the properties using the subsystem name as a
     * prefix.
     *
     * @param properties the properties that include the talon setting
     * @param subsystemName the name of the subsystem that is used in the properties
     * @return a TalonFxSettings with values matching the properties
     */
    public static TalonFxSettings getSettings(Properties properties, String subsystemName) {
      TalonFxSettings talonSettings = new TalonFxSettings();
      talonSettings.canId =
          Integer.parseInt(properties.getProperty(subsystemName + ".talonFXSettings.id"));
      talonSettings.supplyCurrentLimitAmps =
          Double.parseDouble(
              properties.getProperty(subsystemName + ".talonFXSettings.supplyCurrentLimitAmps"));
      talonSettings.supplyCurrentLowerLimitAmps =
          Double.parseDouble(
              properties.getProperty(
                  subsystemName + ".talonFXSettings.supplyCurrentLowerLimitAmps"));
      talonSettings.supplyCurrentLowerTimeSeconds =
          Double.parseDouble(
              properties.getProperty(
                  subsystemName + ".talonFXSettings.supplyCurrentLowerTimeSeconds"));

      talonSettings.statorCurrentLimitAmps =
          Double.parseDouble(
              properties.getProperty(subsystemName + ".talonFXSettings.statorCurrentLimitAmps"));
      talonSettings.enableStatorCurrentLimit =
          Boolean.parseBoolean(
              properties.getProperty(subsystemName + ".talonFXSettings.enableStatorCurrentLimit"));
      talonSettings.peakForwardVoltage =
          MathUtil.clamp(
              Double.parseDouble(
                  properties.getProperty(subsystemName + ".talonFXSettings.peakForwardVoltage")),
              0.0,
              12.0);
      talonSettings.peakReverseVoltage =
          MathUtil.clamp(
              Double.parseDouble(
                  properties.getProperty(subsystemName + ".talonFXSettings.peakReverseVoltage")),
              -12.0,
              0);

      return talonSettings;
    }
  }

  MotorIOBaseSettings motorSettings;

  private final TalonFX motorFx;

  public MotorIOTalonFx(MotorIOBaseSettings motorSettings, TalonFxSettings talonFxSettings) {
    super(motorSettings);
    this.motorSettings = motorSettings;
    motorFx = new TalonFX(talonFxSettings.canId);

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    /* Configure the Talon to use a supply limit of 70 A, and
     * lower to 40 A if we're at 70 A for over 1 second */

    currentLimitsConfigs
        .withSupplyCurrentLowerLimit(
            Units.Amps.of(talonFxSettings.supplyCurrentLowerLimitAmps)) // Default limit of 70 A
        .withSupplyCurrentLimit(
            Units.Amps.of(
                talonFxSettings
                    .supplyCurrentLimitAmps)) // Reduce the limit to 40 A if we've limited to 70
        // A...
        .withSupplyCurrentLowerTime(
            Units.Seconds.of(
                talonFxSettings.supplyCurrentLowerTimeSeconds)) // ...for at least 1 second
        .withSupplyCurrentLimitEnable(true); // And enable it

    currentLimitsConfigs
        .withStatorCurrentLimit(Units.Amps.of(120)) // Limit stator current to 120 A
        .withStatorCurrentLimitEnable(true); // And enable it

    // Peak output of 12 V
    toConfigure
        .Voltage
        .withPeakForwardVoltage(Units.Volts.of(12))
        .withPeakReverseVoltage(Units.Volts.of(-12));

    toConfigure.CurrentLimits = currentLimitsConfigs;

    toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    if (motorSettings.motor.inverted) {
      toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorFx.getConfigurator().apply(toConfigure);
  }

  @Override
  public void setVoltage(double volts) {
    // Inversion is handled in the Talon FX Controller config, always set it to false
    motorFx.setVoltage(calculateSafeVoltage(volts, false));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.appliedVolts = motorFx.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motorFx.getSupplyCurrent().getValueAsDouble();
    // inputs.currentStatorAmps = motorFx.getStatorCurrent().getValueAsDouble();

    inputs.positionRad =
        edu.wpi.first.math.util.Units.rotationsToRadians(
            motorFx.getPosition().getValueAsDouble() / motorSettings.motor.gearing);
    inputs.velocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(
            motorFx.getVelocity().getValueAsDouble() / motorSettings.motor.gearing);

    super.updateInputs(inputs);
  }

  @Override
  public void resetEncoder(double positionRad) {
    motorFx.setPosition(
        edu.wpi.first.math.util.Units.radiansToRotations(
            positionRad * motorSettings.motor.gearing));
  }
}
