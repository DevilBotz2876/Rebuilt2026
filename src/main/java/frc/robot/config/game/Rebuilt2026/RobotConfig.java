package frc.robot.config.game.Rebuilt2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.implementations.drive.DriveBase;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static SendableChooser<Command> autoChooser;
  // TODO: Add VisionSubsystem Declaration

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

    // TODO: Add VisionSubsystem Initialization

    if (stubVision) {
      // TODO: Add VisionSubsystem Settings
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      // TODO: Add VisionSubsystem Simulation Support

      // HACK just to verify autos are visible without connecting to robot
      RobotConfig.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }

    // Send vision-based odometry measurements to drive's odometry calculations
    // vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    if (null != RobotConfig.autoChooser) {
      SmartDashboard.putData("Autonomous", RobotConfig.autoChooser);
    }
  }
}
