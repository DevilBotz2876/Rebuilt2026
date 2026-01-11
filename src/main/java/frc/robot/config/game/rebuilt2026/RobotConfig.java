package frc.robot.config.game.rebuilt2026;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.implementations.drive.DriveBase;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public DriveBase drive;
  public SendableChooser<Command> autoChooser;
  // TODO: Add VisionSubsystem Declaration

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig() {
    // no-args constructor, for now
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
