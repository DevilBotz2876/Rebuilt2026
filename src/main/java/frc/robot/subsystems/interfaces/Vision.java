package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface Vision {
  @FunctionalInterface
  interface VisionMeasurementConsumer {
    void add(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  public interface Camera {
    @AutoLog
    public class CameraInputs {
      public boolean isConnected = false;
      public int[] targetIds = new int[0];
      public double cameraDistanceToTargetMeters = -1;
      public Pose2d cameraPose;
    }

    public static class CameraSettings {
      public int fps;
      public int resWidth;
      public int resHeight;
    }

    public class VisionPoseMeasurement {
      public double timestamp = -1.0;
      public Pose2d robotPose = new Pose2d();
      public int[] targetIds = new int[0];
      public double robotToBestTargetDistanceInMeters = -1.0;
    }

    public void update(CameraInputs inputs);

    public Transform3d getRobotToCamera();

    public String getName();

    public VisionPoseMeasurement[] getVisionPoseMeasurements();

    public CameraSettings getCameraSettings();
  }

  public void updateCamera(int index);

  public void addCamera(Camera camera);

  public List<Camera> getCameras();

  public List<CameraInputsAutoLogged> getCameraInputs();

  public AprilTagFieldLayout getFieldLayout();
}
