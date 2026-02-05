package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class CameraPhoton extends CameraBase {
  private final PhotonCamera camera;
  private final AprilTagFieldLayout tagLayout;
  private VisionPoseMeasurement[] poseMeasurements;

  public CameraPhoton(
      String name,
      Transform3d robotToCamera,
      CameraSettings settings,
      AprilTagFieldLayout tagLayout) {
    super(name, robotToCamera, settings);
    camera = new PhotonCamera(getName());
    this.tagLayout = tagLayout;
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

    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        // test out different things
        System.out.println("Id: " + target.objDetectId);
        System.out.println("Area: " + target.getArea());
        System.out.println("Targets or something" + result.getTargets());
        
    }
      //System.out.println("  Bounding Box X: " + TargetCorner.getMinX());
      //System.out.println("  Bounding Box Y: " + TargetCorner.getMinY());
      inputs.targetIds = new int[0];
      poseMeasurements[0] = new VisionPoseMeasurement();
      return;
    }

    /* 
    // no tags
    /* 
    if (!result.hasTargets()) {
      inputs.targetIds = new int[0];
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
      */

    inputs.cameraDistanceToTargetMeters =
        result
            .getBestTarget()
            .getBestCameraToTarget()
            .getTranslation()
            .getDistance(Translation3d.kZero);

    poseMeasurements[0] = createMeasurement(result, inputs.cameraPose, inputs.targetIds);
  }

  @Override
  public VisionPoseMeasurement[] getVisionPoseMeasurements() {
    return poseMeasurements;
  }

  public PhotonCamera getPhotonCamera() {
    return camera;
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
