package frc.robot.subsystems.controls.combination;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public enum DynamicLocation {
  ORIGIN("Origin", 0.0, 0.0),
  DEPOT("Depot", 0.7348, 6.009),
  OUTPOST("Outpost", 0.7348, 0.650),
  LEFT_BUMP("Left Bump", 4.625594, 5.55244),
  RIGHT_BUMP("Right Bump", 4.625594, 2.516886),
  LEFT_TRENCH("Left Trench", 4.625594, 7.435088),
  RIGHT_TRENCH("Right Trench", 4.625594, 0.634238),
  LEFT_NEUTRAL_ZONE_ALLIANCE_SIDE("Left Neutral Zone Alliance Side", 7.657, 6.67512),
  RIGHT_NEUTRAL_ZONE_ALLIANCE_SIDE("Right Neutral Zone Alliance Side", 7.657, 1.394206),
  LEFT_NEUTRAL_ZONE_CENTER_LINE("Left Neutral Zone Center Line", 8.270494, 6.67512),
  RIGHT_NEUTRAL_ZONE_CENTER_LINE("Right Neutral Zone Center Line", 8.270494, 1.394206),
  LEFT_NEUTRAL_ZONE_OPPONENT_SIDE("Left Neutral Zone Opponent Side", 8.840, 6.67512),
  RIGHT_NEUTRAL_ZONE_OPPONENT_SIDE("Left Neutral Zone Center Line", 8.840, 1.394206),
  HUB("Hub", 4.625594, 4.0346376);

  private String name;
  private double x;
  private double y;

  DynamicLocation(String name, double x, double y) {
    this.name = name;
    this.x = x;
    this.y = y;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public String getName() {
    return name;
  }

  public boolean equal(DynamicLocation location) {
    return this.x == location.getX()
        && this.y == location.getY()
        && this.name == location.getName();
  }

  public static Command createPathfindingToLocationCommand(
      DynamicLocation location, Rotation2d endRot, PathConstraints constraints) {
    return createPathfindingToLocationCommand(
        new Pose2d(location.x, location.y, endRot), constraints, 0.0);
  }

  public static Command createPathfindingToLocationCommand(
      DynamicLocation location,
      Rotation2d endRot,
      PathConstraints constraints,
      double endVelocity) {
    return createPathfindingToLocationCommand(
        new Pose2d(location.x, location.y, endRot), constraints, endVelocity);
  }

  public static Command createPathfindingToLocationCommand(
      Pose2d target, PathConstraints constraints, double endVelocity) {
    return AutoBuilder.pathfindToPoseFlipped(target, constraints, endVelocity);
  }
}
