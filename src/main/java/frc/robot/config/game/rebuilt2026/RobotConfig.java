package frc.robot.config.game.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.config.game.rebuilt2026.tunerConstants.TunerConstants;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveCTRE;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.implementations.vision.camera.CameraPhoton;
import frc.robot.subsystems.implementations.vision.camera.CameraPhotonSim;
import frc.robot.subsystems.interfaces.Vision.Camera.CameraSettings;
import java.util.Optional;
import java.util.Properties;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public DriveBase drive;
  public SendableChooser<Command> autoChooser;
  public VisionSubsystem vision;
  // TODO: Add VisionSubsystem Declaration

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(Properties robotProperties) {
    if (robotProperties.getProperty("robot.drive").equals("ctre")) {
      this.drive = new DriveSwerveCTRE(new TunerConstants(robotProperties));
    } else {
      drive = new DriveBase("BASE");
    }

    // no-args constructor, for now
    vision =
        new VisionSubsystem(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            Optional.of(drive::addVisionMeasurement));

    CameraSettings simCameraSettings = new CameraSettings();
    simCameraSettings.fps = 30;
    simCameraSettings.resWidth = 800;
    simCameraSettings.resHeight = 600;
    // "sim" camera that +12 inch along x axis and pitch of -45 degrees
    vision.addCamera(
        new CameraPhoton(
            "left",
            new Transform3d(
                new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(14.5), Units.inchesToMeters(7)),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(5, Degrees),
                    Angle.ofBaseUnits(-22, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "front",
            new Transform3d(
                new Translation3d(Units.inchesToMeters(14.5), Units.inchesToMeters(0.25), Units.inchesToMeters(3.5)),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "right",
            new Transform3d(
                new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(-14.5), Units.inchesToMeters(7)),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(5, Degrees),
                    Angle.ofBaseUnits(22, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "other1",
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "other2",
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "other3",
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
    vision.addCamera(
        new CameraPhoton(
            "other4",
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees),
                    Angle.ofBaseUnits(0, Degrees))),
            simCameraSettings,
            vision.getFieldLayout()));
  }

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    if (stubDrive) {
      drive = new DriveBase("Stub");
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    // TODO: Add VisionSubsystem Initialization

    if (stubVision) {
      // TODO: Add VisionSubsystem Settings
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      // TODO: Add VisionSubsystem Simulation Support

      // HACK just to verify autos are visible without connecting to robot
      // this.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }
    DriveControls.setupController(drive, mainController);
    // Send vision-based odometry measurements to drive's odometry calculations
    // vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    if (null != this.autoChooser) {
      SmartDashboard.putData("Autonomous", this.autoChooser);
    }
  }
}
