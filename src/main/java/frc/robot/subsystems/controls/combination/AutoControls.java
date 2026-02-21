package frc.robot.subsystems.controls.combination;

import java.util.ArrayList;
import java.util.List;
import java.util.Properties;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI);

        
        NamedCommands.registerCommand("Launch 8 From Known Distance", launch8FuelFromKnownDistance);
        NamedCommands.registerCommand("Launch From x Distance", new WaitCommand(1.0));
        NamedCommands.registerCommand("Drive to Hub", new WaitCommand(0.0));
        NamedCommands.registerCommand("Drive to Outpost", DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.OUTPOST, Rotation2d.kZero, constraints));
        NamedCommands.registerCommand("Drive to Depot", DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.DEPOT, Rotation2d.kZero, constraints));
        
        NamedCommands.registerCommand("Drive to Left Neutral Zone (Alliance Side) Through Left Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Left Neutral Zone (Alliance Side) Through Left Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE, Rotation2d.kCCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Alliance Side) Through Right Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_ALLIANCE_SIDE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Alliance Side) Through Right Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_ALLIANCE_SIDE, Rotation2d.kCCW_90deg, constraints)));

        NamedCommands.registerCommand("Drive to Left Neutral Zone (Center Line) Through Left Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_CENTER_LINE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Left Neutral Zone (Center Line) Through Left Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE, Rotation2d.kCCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Center Line) Through Right Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_CENTER_LINE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Center Line) Through Right Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_CENTER_LINE, Rotation2d.kCCW_90deg, constraints)));

        NamedCommands.registerCommand("Drive to Left Neutral Zone (Opponent Side) Through Left Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_OPPONENT_SIDE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Left Neutral Zone (Opponent Side) Through Left Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.LEFT_NEUTRAL_ZONE_OPPONENT_SIDE, Rotation2d.kCCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Opponent Side) Through Right Trench", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_OPPONENT_SIDE, Rotation2d.kCW_90deg, constraints)));
        NamedCommands.registerCommand("Drive to Right Neutral Zone (Opponent Side) Through Right Bump", new SequentialCommandGroup(DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0), DynamicLocation.createPathfindingToLocationCommand(DynamicLocation.RIGHT_NEUTRAL_ZONE_OPPONENT_SIDE, Rotation2d.kCCW_90deg, constraints)));

        NamedCommands.registerCommand("Intake Fuel", new SequentialCommandGroup(intakeIn, new WaitCommand(2.0), stopIntake));
        NamedCommands.registerCommand("Intake Fuel from Depot", new WaitCommand(1.0));
        NamedCommands.registerCommand("Intake Fuel from Outpost", new WaitCommand(1.0));
        
    }
}