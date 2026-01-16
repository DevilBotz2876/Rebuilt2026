package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class CameraPhotonSim extends CameraPhoton {
  private final SimCameraProperties simCameraProperties;
  private final PhotonCameraSim simCamera;
  private static VisionSystemSim simVision;
  private final Supplier<Pose2d> poseSupplier;

  public CameraPhotonSim(
      String name,
      Transform3d robotToCamera,
      CameraSettings settings,
      AprilTagFieldLayout tagLayout,
      Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera, settings, tagLayout);
    // from last year
    simCameraProperties = new SimCameraProperties();
    simCameraProperties.setCalibration(
        settings.resWidth, settings.resHeight, Rotation2d.fromDegrees(70));
    simCameraProperties.setFPS(120);
    simCameraProperties.setAvgLatencyMs(50);
    simCameraProperties.setLatencyStdDevMs(15);

    simCamera = new PhotonCameraSim(getPhotonCamera(), simCameraProperties, tagLayout);

    this.poseSupplier = poseSupplier;

    // on fist sim camera, make the VisionSystemSim, rest of simulated camera use the same system
    if (simVision == null) {
      simVision = new VisionSystemSim("main");
      simVision.addAprilTags(tagLayout);
    }

    simVision.addCamera(simCamera, robotToCamera);
    simCamera.enableDrawWireframe(true);
  }
  ;

  @Override
  public void update(CameraInputs inputs) {
    simVision.update(poseSupplier.get());
    super.update(inputs);
  }
}
