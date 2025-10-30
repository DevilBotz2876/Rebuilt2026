package frc.robot.config.game.Rebuilt2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static SendableChooser<Command> autoChooser;
  public static VisionSubsystem vision;

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    if (stubDrive) {
      drive = new DriveBase("Stub");
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    vision =
        new VisionSubsystem(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

    if (stubVision) {
      if (Robot.isSimulation()) {
        vision.addCamera(
            new Camera(
                "rear_cam",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(0.295),
                        Units.inchesToMeters(-11.443),
                        Units.inchesToMeters(39.663)),
                    new Rotation3d(
                        Units.degreesToRadians(12),
                        Units.degreesToRadians(-33),
                        Units.degreesToRadians(170)))));
      }
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), true);

      // HACK just to verify autos are visible without connecting to robot
      RobotConfig.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }

    // Send vision-based odometry measurements to drive's odometry calculations
    vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    if (null != RobotConfig.autoChooser) {
      SmartDashboard.putData("Autonomous", RobotConfig.autoChooser);
    }
  }
}
