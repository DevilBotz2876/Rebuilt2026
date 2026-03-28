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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.common.flywheel.FlywheelToVelocity;
import frc.robot.commands.common.motor.MotorRunVoltageCommand;
import frc.robot.subsystems.controls.combination.DriverControls.DriverControlsSettings;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
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
      Flywheel intake,
      Flywheel shooter,
      Flywheel indexer,
      Flywheel conveyor,
      Arm intakeArm,
      AutoRoutineSettings autoSettings,
      DriverControlsSettings driverSettings) {
    // Commands to stop subsystem
    Command stopShooter = new MotorRunVoltageCommand((Motor) shooter, () -> 0.0);
    Command stopIndexer = new MotorRunVoltageCommand((Motor) indexer, () -> 0.0);
    Command stopConveyor = new MotorRunVoltageCommand((Motor) conveyor, () -> 0.0);
    Command stopIntake = new MotorRunVoltageCommand((Motor) intake, () -> 0.0);
    Command stopIntakeArm = new MotorRunVoltageCommand((Motor) intakeArm, () -> 0.0);

    // Intake Commands
    Command intakeIn =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> driverSettings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM),
            new MotorRunVoltageCommand(
                (Motor) intakeArm, () -> driverSettings.intakeArmWhileInakingVolts));

    Command intakeOut =
        new ParallelCommandGroup(
            new FlywheelToVelocity(intake, () -> -driverSettings.intakeRPM),
            new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorReverseRPM));

    // Auto commands
    // launch for the (time of one ball + timeout) * 8 balls
    Command launchSequentialParallelSmartDashBoard =
        new SequentialCommandGroup(
            new FlywheelToVelocity(
                shooter,
                () ->
                    SmartDashboard.getNumber(
                        "Controls/launchShooterRPM", driverSettings.shooterCurrentLaunchRPM)),
            new ParallelCommandGroup(
                new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
                new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM)));

    // Command launchOutpost =
    //     new SequentialCommandGroup(
    //         new FlywheelToVelocity(shooter, () -> driverSettings.shooterOutpostLaunchRPM),
    //         new ParallelCommandGroup(
    //             new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
    //             new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM)));

    // Command launchTrench =
    //     new SequentialCommandGroup(
    //         new FlywheelToVelocity(shooter, () -> driverSettings.shooterTrenchLaunchRPM),
    //         new ParallelCommandGroup(
    //             new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
    //             new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM)));

    // Command launchAgainstHub =
    //     new SequentialCommandGroup(
    //         new FlywheelToVelocity(shooter, () -> driverSettings.shooterAgainstHubLaunchRPM),
    //         new ParallelCommandGroup(
    //             new FlywheelToVelocity(indexer, () -> driverSettings.indexerLaunchRPM),
    //             new FlywheelToVelocity(conveyor, () -> driverSettings.conveyorLaunchRPM)));

    Command launch8FuelSmartDashboard =
        new SequentialCommandGroup(
            launchSequentialParallelSmartDashBoard.asProxy(),
            new WaitCommand(
                (autoSettings.launchOneFuelSeconds + autoSettings.launchOneTimeoutSeconds) * 8),
            stopShooter.asProxy().withTimeout(0.25),
            stopConveyor.asProxy().withTimeout(0.25),
            stopIndexer.asProxy().withTimeout(0.25));

    Command stopLaunching =
        new SequentialCommandGroup(
            stopShooter.asProxy().withTimeout(0.25),
            stopConveyor.asProxy().withTimeout(0.25),
            stopIndexer.asProxy().withTimeout(0.25));

    AutoControls.drive = drive;
    SmartDashboard.putNumber("Auto/rad", 2.05);
    PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI);

    Command DeployerVoltagePlus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> driverSettings.intakeArmVolts);
    Command DeployerVoltageMinus =
        new MotorRunVoltageCommand((Motor) intakeArm, () -> -driverSettings.intakeArmVolts);

    NamedCommands.registerCommand(
        "Deploy Intake",
        new SequentialCommandGroup(
            DeployerVoltageMinus.asProxy(), new WaitCommand(2), stopIntakeArm.asProxy()));
    NamedCommands.registerCommand(
        "Deployer Up",
        new SequentialCommandGroup(
            DeployerVoltagePlus.asProxy(), new WaitCommand(1), stopIntakeArm.asProxy()));
    NamedCommands.registerCommand(
        "Launch 8 From Known Distance SDB", launch8FuelSmartDashboard.asProxy().withTimeout(4));
    NamedCommands.registerCommand(
        "Launch From Depot", launch8FuelSmartDashboard.asProxy().withTimeout(4));
    NamedCommands.registerCommand(
        "Launch From Center Hub Path",
        new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putNumber("Controls/launchShooterRPM", 2720)),
            launchSequentialParallelSmartDashBoard.asProxy()));
    NamedCommands.registerCommand(
        "Launch From Neutral Path",
        new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putNumber("Controls/launchShooterRPM", 3725)),
            launchSequentialParallelSmartDashBoard.asProxy()));
    NamedCommands.registerCommand("Stop Launching", stopLaunching.asProxy().withTimeout(0.5));

    NamedCommands.registerCommand(
        "Start Shooter from Radius Distance",
        Commands.defer(
            () -> new FlywheelToVelocity(shooter, () -> driverSettings.shooterCurrentLaunchRPM),
            Set.of((Subsystem) shooter)));
    NamedCommands.registerCommand("Start Shooter from Current Distance", new WaitCommand(1.0));
    NamedCommands.registerCommand(
        "Start Shooter (3800RPM)", new FlywheelToVelocity(shooter, () -> 3800));
    NamedCommands.registerCommand(
        "Start Shooter (3665RPM)", new FlywheelToVelocity(shooter, () -> 3665));
    NamedCommands.registerCommand(
        "Start Shooter (3250RPM)", new FlywheelToVelocity(shooter, () -> 3250));
    NamedCommands.registerCommand(
        "Start Shooter (3600RPM)", new FlywheelToVelocity(shooter, () -> 3600));
    NamedCommands.registerCommand("Stop Shooter", stopShooter.asProxy().withTimeout(0.25));

    NamedCommands.registerCommand(
        "Drive to Hub (Radius)",
        Commands.defer(
            () ->
                DynamicLocation.createPathfindingToLocationCommand(
                    AutoControls.getGoToRadiusPose2d(drive), constraints, 0),
            Set.of((Subsystem) drive)));
    NamedCommands.registerCommand(
        "Drive to Outpost",
        DynamicLocation.createPathfindingToLocationCommand(
                DynamicLocation.OUTPOST, Rotation2d.k180deg, constraints)
            .withTimeout(4));
    NamedCommands.registerCommand(
        "Drive to Depot",
        DynamicLocation.createPathfindingToLocationCommand(
            DynamicLocation.DEPOT, Rotation2d.k180deg, constraints));
    NamedCommands.registerCommand("Intake In", intakeIn.asProxy().withTimeout(5));
    NamedCommands.registerCommand("Stop Intake", stopIntake.asProxy().withTimeout(0.1));
    NamedCommands.registerCommand(
        "Rotate to score",
        Commands.defer(
            () ->
                DynamicLocation.createPathfindingToLocationCommand(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        Rotation2d.fromRadians(
                            getHubScoreRotation(drive.getPose().getX(), drive.getPose().getY()))),
                    constraints,
                    0),
            Set.of((Subsystem) drive)));
  }

  public static Pose2d getGoToRadiusPose2d(Drive drive) {
    double radius = SmartDashboard.getNumber("Auto/rad", -1);
    if (radius < 1.18) {
      Elastic.sendNotification(
          new Notification()
              .withTitle("INVAILD RADIUS")
              .withDescription("The radius was less than 1.18. Will not move"));
      return drive.getPose();
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d drivePose = drive.getPose();
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
            ? new Pose2d(x2, y2, Rotation2d.fromRadians(getHubScoreRotation(x2, y2)))
            : new Pose2d(x1, y1, Rotation2d.fromRadians(getHubScoreRotation(x1, y1)));
    }
    return dist1 < dist2
        ? new Pose2d(x1, y1, Rotation2d.fromRadians(getHubScoreRotation(x1, y1)))
        : new Pose2d(x2, y2, Rotation2d.fromRadians(getHubScoreRotation(x2, y2)));
  }

  public static double calcShooterSpeedFromRad() {
    return 4000;
    // calcShooterSpeedFromDistance(SmartDashboard.getNumber("Auto/rad", -1));
  }

  public static double getHubScoreRotation(double x, double y) {
    double leg1 = DynamicLocation.HUB.getY() - y;
    double leg2 = DynamicLocation.HUB.getX() - x;

    return Math.atan(leg1 / leg2) + Math.PI;
  }
}
