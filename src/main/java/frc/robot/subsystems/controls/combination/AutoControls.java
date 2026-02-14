package frc.robot.subsystems.controls.combination;

import java.util.Properties;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.controls.combination.DriverControls.DriverControlsSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class AutoControls {
    public static class AutoRoutineSettings {
        public double launchOneFuelSeconds;
        public double launchOneTimeoutSeconds;
        
        public double intakeDepotTimeoutSeconds;
        public double intakeOutpostTimeoutSeconds;

        public double dynamicPathTimeoutSeconds;

        public static AutoRoutineSettings getAutoRoutineSettings(Properties properties) {
            AutoRoutineSettings settings = new AutoRoutineSettings();
            settings.launchOneFuelSeconds = Double.parseDouble(properties.getProperty("auto.launchOneFuelSeconds"));
            settings.launchOneTimeoutSeconds = Double.parseDouble(properties.getProperty("auto.launchOneTimeoutSeconds"));
            settings.intakeDepotTimeoutSeconds = Double.parseDouble(properties.getProperty("auto.intakeDepotTimeoutSeconds"));
            settings.intakeOutpostTimeoutSeconds = Double.parseDouble(properties.getProperty("auto.intakeOutpostTimeoutSeconds"));
            settings.dynamicPathTimeoutSeconds = Double.parseDouble(properties.getProperty("auto.dynamicPathTimeoutSeconds"));
            return settings;
        }
    }
    
    public static void registerNamedCommands(Drive drive,
      Flywheel topIntake,
      Flywheel bottomIntake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      AutoRoutineSettings autoSettings, DriverControlsSettings driverSettings) {
        // Commands to stop subsystem
    Command stopShooter = new MotorRunVoltageCommand((Motor) shooter, () -> 0.0);
    Command stopIndexer = new MotorRunVoltageCommand((Motor) indexer, () -> 0.0);
    Command stopConveyor = new MotorRunVoltageCommand((Motor) conveyor, () -> 0.0);
    Command stopTopIntake = new MotorRunVoltageCommand((Motor) topIntake, () -> 0.0);
    Command stopBottomIntake = new MotorRunVoltageCommand((Motor) bottomIntake, () -> 0.0);

    // Launching related commands
    Command launchSequentialParallel =
        new SequentialCommandGroup(
            new FlywheelToVelocity(shooter, () -> driverSettings.shooterLaunchRPM),
            new ParallelCommandGroup(
                new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
                new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM)));

    Command launchAllSequential =
        new SequentialCommandGroup(
            new FlywheelToVelocity(shooter, () -> driverSettings.shooterLaunchRPM),
            new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
            new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM));

    Command stopLaunch = stopShooter.alongWith(stopIndexer, stopConveyor);

    // Intake Commands

    Command intakeIn =
        new ParallelCommandGroup(
            new FlywheelToVelocity(topIntake, () -> driverSettings.intakeRPM),
            new FlywheelToVelocity(bottomIntake, () -> driverSettings.intakeRPM));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(topIntake, () -> -driverSettings.intakeRPM),
            new FlywheelToVelocity(bottomIntake, () -> -driverSettings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorReverseRPM));

    Command stopIntake = stopTopIntake.alongWith(stopBottomIntake);


    // Auto commands
        // launch for the (time of one ball + timeout) * 8 balls
        Command launch8FuelFromKnownDistance = new SequentialCommandGroup(
            launchAllSequential,
            new WaitCommand((autoSettings.launchOneFuelSeconds + autoSettings.launchOneTimeoutSeconds) * 8),
            stopLaunch
        );

        NamedCommands.registerCommand("Launch 8 From Known Distance", launch8FuelFromKnownDistance);
        NamedCommands.registerCommand("Launch From x Distance", new WaitCommand(1.0));
        NamedCommands.registerCommand("Driving to Hub", new WaitCommand(1.0));
        NamedCommands.registerCommand("Driving to Outpost", new WaitCommand(1.0));
        NamedCommands.registerCommand("Driving to Depot", new WaitCommand(1.0));
        NamedCommands.registerCommand("Driving to Neutral Zone", new WaitCommand(1.0));
        NamedCommands.registerCommand("Intake Fuel", new WaitCommand(1.0));
        NamedCommands.registerCommand("Intake Fuel from Depot", new WaitCommand(1.0));
        NamedCommands.registerCommand("Intake Fuel from Outpost", new WaitCommand(1.0));
        
        
        
        
        
        
        
    }
}
