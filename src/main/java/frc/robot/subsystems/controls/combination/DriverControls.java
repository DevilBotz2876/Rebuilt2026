package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class DriverControls {
    public static void setupController(Drive drive, Flywheel topIntake, Flywheel bottomIntake, Flywheel shooter, Flywheel indexer, Flywheel conveyor) {}

    public static void setupSmartDashboardControl(Motor motor) {
        SubsystemBase motorSubsystem = (SubsystemBase) motor;
        SmartDashboard.putString("Selected Subsystems/Selected", "UNKNOWN");
        SmartDashboard.putData(
            "Selected Subsystems/Select " + motorSubsystem.getName(),
            new InstantCommand(
                () ->
                    SmartDashboard.putString(
                        "Selected Subsystems/Selected", motorSubsystem.getName())));
    }
}
