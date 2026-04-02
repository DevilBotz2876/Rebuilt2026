package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.subsystems.interfaces.Drive;

public class DriveControls {
  public static void setupController(Drive drive, CommandXboxController controller) {
    SubsystemBase driveSubsystem = (SubsystemBase) drive;
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY() * (drive.isFieldOrientedDrive() ? 1 : -1), 0.05), // Robot Strafe Front/Back
            () -> MathUtil.applyDeadband(-controller.getLeftX() * (drive.isFieldOrientedDrive() ? 1 : -1), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05))); // Robot Rotate
    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *
     *    Start Button = Toggle Drive Orientation
     */
    controller
        .back()
        .and(controller.start().negate())
        .onTrue(new InstantCommand(() -> drive.resetOdometry()));

    controller
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drive.setFieldOrientedDrive(
                        !drive.isFieldOrientedDrive()))); // Toggle Drive Orientation

    // Reset Field Centric Heading
    SmartDashboard.putData(
        driveSubsystem.getName() + "/Reset Field Centric Heading",
        drive.resetFieldCentricHeading().onlyIf(() -> drive.isFieldOrientedDrive()));
  }
}
