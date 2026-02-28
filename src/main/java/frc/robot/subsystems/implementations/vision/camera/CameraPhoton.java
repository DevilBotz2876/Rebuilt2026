package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import java.util.Optional;
import java.util.Properties;
import java.util.function.ToIntFunction;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraPhoton extends CameraBase {

  public static CameraPhoton createCameraPhoton(
      Properties properties, String name, AprilTagFieldLayout layout) {

    Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.xMeters")),
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.yMeters")),
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.zMeters"))),
            new Rotation3d(
                Units.degreesToRadians(Double.parseDouble(properties.getProperty(name + ".robotToCamera.rollDegrees"))),
                Units.degreesToRadians(Double.parseDouble(properties.getProperty(name + ".robotToCamera.pitchDegrees"))),
                Units.degreesToRadians(Double.parseDouble(properties.getProperty(name + ".robotToCamera.yawDegrees")))));

    CameraSettings settings = CameraSettings.getSettings(properties, name);

    return new CameraPhoton(name, robotToCamera, settings, layout);
  }

  private final PhotonCamera camera;
  private final AprilTagFieldLayout tagLayout;
  private VisionPoseMeasurement[] poseMeasurements;
  private final PhotonPoseEstimator photonPoseEstimator;

  public CameraPhoton(
      String name,
      Transform3d robotToCamera,
      CameraSettings settings,
      AprilTagFieldLayout tagLayout) {
    super(name, robotToCamera, settings);
    camera = new PhotonCamera(getName());
    this.tagLayout = tagLayout;
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);
    SmartDashboard.putNumber(getName() + "/x", robotToCamera.getX());
    SmartDashboard.putNumber(getName() + "/y", robotToCamera.getY());
    SmartDashboard.putNumber(getName() + "/z", robotToCamera.getZ());
    SmartDashboard.putNumber(getName() + "/roll", Units.radiansToDegrees(robotToCamera.getRotation().getX()));
    SmartDashboard.putNumber(getName() + "/pitch", Units.radiansToDegrees(robotToCamera.getRotation().getY()));
    SmartDashboard.putNumber(getName() + "/yaw", Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
}

  @Override
  public void update(CameraInputs inputs) {
    inputs.isConnected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.isEmpty() || !inputs.isConnected) {
      this.poseMeasurements = new VisionPoseMeasurement[0];
      return;
    }

    PhotonPipelineResult result = results.get(results.size() - 1); // get latest result
    this.poseMeasurements = new VisionPoseMeasurement[1];

    // no tags
    if (!result.hasTargets()) {
      inputs.targetIds = new int[0];
      inputs.cameraDistanceToTargetMeters = -1.0;
      poseMeasurements[0] = new VisionPoseMeasurement();
      return;
    }

    // 2+ tags
    if (result.multitagResult.isPresent()) {
      MultiTargetPNPResult multitagResult = result.multitagResult.get();
      inputs.cameraPose =
          new Pose3d(Translation3d.kZero, Rotation3d.kZero)
              .plus(multitagResult.estimatedPose.best)
              .toPose2d();
      inputs.targetIds =
          multitagResult.fiducialIDsUsed.stream().mapToInt(Short::toUnsignedInt).toArray();
      poseMeasurements[0] = createMeasurement(result);
      return;
    }

    // one tag
    else {
      // use location on field to determine pose
      Pose3d aprilTagPose = tagLayout.getTagPose(result.getBestTarget().fiducialId).get();
      inputs.cameraPose =
          new Pose3d(aprilTagPose.getTranslation(), aprilTagPose.getRotation())
              .plus(result.getBestTarget().bestCameraToTarget.inverse())
              .toPose2d();
      inputs.targetIds = new int[1];
    }

    inputs.cameraDistanceToTargetMeters =
        result
            .getBestTarget()
            .getBestCameraToTarget()
            .getTranslation()
            .getDistance(Translation3d.kZero);

    // poseMeasurements[0] = createMeasurement(result, inputs.cameraPose, inputs.targetIds);
    photonPoseEstimator.setRobotToCameraTransform(new Transform3d(new Translation3d(SmartDashboard.getNumber(getName() + "/x", 0),SmartDashboard.getNumber(getName() + "/y", 0),SmartDashboard.getNumber(getName() + "/z", 0)),new Rotation3d(Units.degreesToRadians(SmartDashboard.getNumber(getName() + "/roll", 0)), Units.degreesToRadians(SmartDashboard.getNumber(getName() + "/pitch", 0)), Units.degreesToRadians(SmartDashboard.getNumber(getName() + "/yaw", 0)))));
    poseMeasurements[0] = createMeasurement(result);
    inputs.targetIds = poseMeasurements[0].targetIds;
  }

  @Override
  public VisionPoseMeasurement[] getVisionPoseMeasurements() {
    return poseMeasurements;
  }

  public PhotonCamera getPhotonCamera() {
    return camera;
  }

  private VisionPoseMeasurement createMeasurement(PhotonPipelineResult result) {
    VisionPoseMeasurement measurement = new VisionPoseMeasurement();
    Optional<EstimatedRobotPose> estPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);
    if (estPose.isEmpty()) {
      estPose = photonPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    if (estPose.isEmpty()) {
      return measurement;
    }

    measurement.targetIds =
        estPose.get().targetsUsed.stream()
            .mapToInt(
                new ToIntFunction<PhotonTrackedTarget>() {
                  public int applyAsInt(PhotonTrackedTarget value) {
                    return value.fiducialId;
                  }
                })
            .toArray();
    measurement.timestamp = estPose.get().timestampSeconds;

    Transform3d robotToCamera = getRobotToCamera();
    measurement.robotPose = estPose.get().estimatedPose.toPose2d();
    Logger.recordOutput("Vision/backRight/cameraPose", estPose.get().estimatedPose.plus(robotToCamera));
    // robot to camera + camera to target = robot to target
    // TODO: Determine best start location for calculated distance
    // the distance should be measured at a place that is easy to verify in person
    measurement.robotToBestTargetDistanceInMeters =
        robotToCamera
            .plus(result.getBestTarget().getBestCameraToTarget())
            .getTranslation()
            .getDistance(Translation3d.kZero);

    return measurement;
  }

  private VisionPoseMeasurement createMeasurement(
      PhotonPipelineResult result, Pose2d cameraPose, int[] targetIds) {
    VisionPoseMeasurement measurement = new VisionPoseMeasurement();

    measurement.targetIds = targetIds;
    measurement.timestamp = result.getTimestampSeconds();

    Transform3d robotToCamera = getRobotToCamera();
    measurement.robotPose =
        cameraPose.transformBy(
            (new Transform2d(
                    robotToCamera.getTranslation().toTranslation2d(),
                    robotToCamera.getRotation().toRotation2d()))
                .inverse());

    // robot to camera + camera to target = robot to target
    // TODO: Determine best start location for calculated distance
    // the distance should be measured at a place that is easy to verify in person
    measurement.robotToBestTargetDistanceInMeters =
        robotToCamera
            .plus(result.getBestTarget().getBestCameraToTarget())
            .getTranslation()
            .getDistance(Translation3d.kZero);

    return measurement;
  }
}
