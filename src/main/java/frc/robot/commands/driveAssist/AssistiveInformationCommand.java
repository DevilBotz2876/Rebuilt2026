package frc.robot.commands.driveAssist;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.controls.combination.DriverControls.DriverControlsSettings;
import frc.robot.subsystems.controls.combination.DynamicLocation;
import frc.robot.subsystems.interfaces.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AssistiveInformationCommand extends Command {
  private Drive drive;
  private DriverControlsSettings settings;
  private DoubleSupplier targetRPM;
  private Pose2d HUB_POSE;
  private final double validDistanceTolerance = 0.2;

  /** Calculates and added the need rotation and need distance to score */
  public AssistiveInformationCommand(Drive drive, DriverControlsSettings settings) {
    this.drive = drive;
    this.settings = settings;
    // this.targetRPM = targetRPM;

    HUB_POSE = new Pose2d(DynamicLocation.HUB.getX(), DynamicLocation.HUB.getY(), Rotation2d.kZero);

    addRequirements();
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        HUB_POSE =
            FlippingUtil.flipFieldPose(
                new Pose2d(
                    DynamicLocation.HUB.getX(), DynamicLocation.HUB.getY(), Rotation2d.kZero));
        return;
      }
    }
    SmartDashboard.putNumber("Controls/Needed Rotation", 0);
    SmartDashboard.putNumber("Controls/Distance from Hub", 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber(
        "Controls/Needed Rotation",
        Units.radiansToDegrees(
            drive.getPose().getRotation().getRadians()
                - getHubScoreRotation(drive.getPose().getX(), drive.getPose().getY())));

    // double shooterTargetRPM = 0;
    // int correctionNeed = -2;
    // double distanceAwayCorrectDistance = 0;
    double distanceFromLocation =
        HUB_POSE.getTranslation().getDistance(drive.getPose().getTranslation());
    // if(shooterTargetRPM == settings.shooterAgainstHubLaunchRPM) {
    //     distanceAwayCorrectDistance = 1.2203852094195 - distanceFromLocation;
    // } else if(shooterTargetRPM == settings.shooterDepotLaunchRPM) {
    //     distanceAwayCorrectDistance = 4.4002435855495 - distanceFromLocation;
    // } else if(shooterTargetRPM == settings.shooterTrenchLaunchRPM) {
    //     distanceAwayCorrectDistance = 3.1990500006361 - distanceFromLocation;
    // } else if(shooterTargetRPM == settings.shooterOutpostLaunchRPM) {
    //     distanceAwayCorrectDistance = 5.1404019413437 - distanceFromLocation;
    // } else {
    //   distanceAwayCorrectDistance = 0;
    // }
    // correctionNeed = Math.abs(distanceAwayCorrectDistance) <= validDistanceTolerance ? 0 : (int)
    // Math.signum(distanceAwayCorrectDistance);
    SmartDashboard.putNumber("Controls/Distance from Hub", distanceFromLocation);
  }

  private double getHubScoreRotation(double x, double y) {
    double leg1 = HUB_POSE.getY() - y;
    double leg2 = HUB_POSE.getX() - x;

    return Math.atan(leg1 / leg2);
  }
}
