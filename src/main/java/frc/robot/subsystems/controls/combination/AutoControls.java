package frc.robot.subsystems.controls.combination;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.controls.combination.DriverControls.DriverControlsSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;
import java.util.Optional;
import java.util.Properties;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoControls {
  public static class AutoRoutineSettings {
    public double launchOneFuelSeconds;
    public double launchOneTimeoutSeconds;

    public double intakeDepotTimeoutSeconds;
    public double intakeOutpostTimeoutSeconds;

    public double dynamicPathTimeoutSeconds;

    public static AutoRoutineSettings getAutoRoutineSettings(Properties properties) {
      AutoRoutineSettings settings = new AutoRoutineSettings();
      settings.launchOneFuelSeconds =
          Double.parseDouble(properties.getProperty("auto.launchOneFuelSeconds"));
      settings.launchOneTimeoutSeconds =
          Double.parseDouble(properties.getProperty("auto.launchOneTimeoutSeconds"));
      settings.intakeDepotTimeoutSeconds =
          Double.parseDouble(properties.getProperty("auto.intakeDepotTimeoutSeconds"));
      settings.intakeOutpostTimeoutSeconds =
          Double.parseDouble(properties.getProperty("auto.intakeOutpostTimeoutSeconds"));
      settings.dynamicPathTimeoutSeconds =
          Double.parseDouble(properties.getProperty("auto.dynamicPathTimeoutSeconds"));
      return settings;
    }
  }

  private static Drive drive = null;
  @AutoLogOutput private static Pose2d pose = new Pose2d();

  public static void registerNamedCommands(
      Drive drive,
      Flywheel topIntake,
      Flywheel bottomIntake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      AutoRoutineSettings autoSettings,
      DriverControlsSettings driverSettings) {
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
    Command launch8FuelFromKnownDistance =
        new SequentialCommandGroup(
            launchAllSequential,
            new WaitCommand(
                (autoSettings.launchOneFuelSeconds + autoSettings.launchOneTimeoutSeconds) * 8),
            stopLaunch);

    AutoControls.drive = drive;
    SmartDashboard.putNumber("Auto/rad", 2.05);
    PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI);

    NamedCommands.registerCommand("Launch 8 From Known Distance", launch8FuelFromKnownDistance);
    NamedCommands.registerCommand("Launch From x Distance", new WaitCommand(1.0));
    NamedCommands.registerCommand(
        "Drive to Hub (Radius)",
        Commands.defer(
            () ->
                DynamicLocation.createPathfindingToLocationCommand(
                    AutoControls.getGoToRadiusPose2d(), constraints, 0),
            Set.of((Subsystem) drive)));
    // new SequentialCommandGroup(
    // new InstantCommand(() -> {SmartDashboard.putNumber("Auto/RadPose/x",
    // AutoControls.getGoToRadiusPose2d().getX());
    // SmartDashboard.putNumber("Auto/RadPose/y", AutoControls.getGoToRadiusPose2d().getY());
    // System.out.println("x: " + AutoControls.pose.getX() + " y: " + AutoControls.pose.getY());}),
    // DynamicLocation.createPathfindingToLocationCommand(new
    // Pose2d(SmartDashboard.getNumber("Auto/RadPose/x", 0),
    // SmartDashboard.getNumber("Auto/RadPose/x", 0), Rotation2d.kZero), constraints, 0))
    // );
    NamedCommands.registerCommand(
        "Drive to Outpost",
        DynamicLocation.createPathfindingToLocationCommand(
            DynamicLocation.OUTPOST, Rotation2d.kZero, constraints));
    NamedCommands.registerCommand(
        "Drive to Depot",
        DynamicLocation.createPathfindingToLocationCommand(
            DynamicLocation.DEPOT, Rotation2d.kZero, constraints));

    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Alliance Side) Through Left Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE,
                Rotation2d.kCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Alliance Side) Through Left Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE,
                Rotation2d.kCCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Alliance Side) Through Right Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_ALLIANCE_SIDE,
                Rotation2d.kCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Alliance Side) Through Right Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_ALLIANCE_SIDE,
                Rotation2d.kCCW_90deg,
                constraints)));

    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Center Line) Through Left Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_CENTER_LINE, Rotation2d.kCW_90deg, constraints)));
    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Center Line) Through Left Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE,
                Rotation2d.kCCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Center Line) Through Right Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_CENTER_LINE,
                Rotation2d.kCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Center Line) Through Right Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_CENTER_LINE,
                Rotation2d.kCCW_90deg,
                constraints)));

    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Opponent Side) Through Left Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_TRENCH, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_OPPONENT_SIDE,
                Rotation2d.kCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Left Neutral Zone (Opponent Side) Through Left Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_BUMP, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.LEFT_NEUTRAL_ZONE_OPPONENT_SIDE,
                Rotation2d.kCCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Opponent Side) Through Right Trench",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_TRENCH, Rotation2d.fromDegrees(30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_OPPONENT_SIDE,
                Rotation2d.kCW_90deg,
                constraints)));
    NamedCommands.registerCommand(
        "Drive to Right Neutral Zone (Opponent Side) Through Right Bump",
        new SequentialCommandGroup(
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_BUMP, Rotation2d.fromDegrees(-30), constraints, 1.0),
            DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.RIGHT_NEUTRAL_ZONE_OPPONENT_SIDE,
                Rotation2d.kCCW_90deg,
                constraints)));

    NamedCommands.registerCommand(
        "Intake Fuel", new SequentialCommandGroup(intakeIn, new WaitCommand(2.0), stopIntake));
    NamedCommands.registerCommand("Intake Fuel from Depot", new WaitCommand(1.0));
    NamedCommands.registerCommand("Intake Fuel from Outpost", new WaitCommand(1.0));
  }

  public static Pose2d getGoToRadiusPose2d() {
    double radius = SmartDashboard.getNumber("Auto/rad", -1);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d drivePose = AutoControls.drive.getPose();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) drivePose = FlippingUtil.flipFieldPose(drivePose);
    }

    double x1, y1, x2, y2, dist1, dist2;

    if (drivePose.getX() - DynamicLocation.HUB.getX() == 0) {
      x1 = drivePose.getX();
      y1 = DynamicLocation.HUB.getY() + radius;
      x2 = drivePose.getX();
      y2 = DynamicLocation.HUB.getY() - radius;
    } else {
      double slope =
          (drivePose.getY() - DynamicLocation.HUB.getY())
              / (drivePose.getX() - DynamicLocation.HUB.getX());
      x1 = (radius / Math.sqrt(1 + Math.pow(slope, 2))) + DynamicLocation.HUB.getX();
      x2 = (radius / -Math.sqrt(1 + Math.pow(slope, 2))) + DynamicLocation.HUB.getX();
      y1 = ((radius * slope) / Math.sqrt(1 + Math.pow(slope, 2))) + DynamicLocation.HUB.getY();
      y2 = ((radius * slope) / -Math.sqrt(1 + Math.pow(slope, 2))) + DynamicLocation.HUB.getY();
    }

    dist1 = drive.getPose().getTranslation().getDistance(new Translation2d(x1, y1));
    dist2 = drive.getPose().getTranslation().getDistance(new Translation2d(x2, y2));

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red)
        return dist1 < dist2
            ? new Pose2d(x2, y2, Rotation2d.fromDegrees(0))
            : new Pose2d(x1, y1, Rotation2d.fromDegrees(0));
    }
    return dist1 < dist2
        ? new Pose2d(x1, y1, Rotation2d.fromDegrees(0))
        : new Pose2d(x2, y2, Rotation2d.fromDegrees(0));
  }
}
