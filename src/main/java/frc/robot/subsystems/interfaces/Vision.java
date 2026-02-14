package frc.robot.subsystems.interfaces;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Properties;
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

      public static CameraSettings getCameraSettings(Properties properties, String name) {
        CameraSettings settings = new CameraSettings();
        settings.fps = Integer.parseInt(properties.getProperty(name + ".settings.fps"));
        settings.resWidth = Integer.parseInt(properties.getProperty(name + ".settings.resWidth"));
        settings.resHeight = Integer.parseInt(properties.getProperty(name + ".settings.resHeight"));

        return settings;
      }
    }

    public class VisionPoseMeasurement {
      public double timestamp = -1.0;
      public Pose2d robotPose = new Pose2d();
      public int[] targetIds = new int[0];
      public double robotToBestTargetDistanceInMeters = -1.0;
    }

    /**
     * Updates the camera, creates pose measurement for that cycle and populates the inputs
     *
     * @param inputs the inputs associatied with the camera
     */
    public void update(CameraInputs inputs);

    /**
     * Returns the 3D transform from the robot to the camera
     *
     * @return the transform from the robot to the camera
     */
    public Transform3d getRobotToCamera();

    /**
     * Returns the name of the camera
     *
     * @return the name
     */
    public String getName();

    /**
     * Returns the array of vision pose measurement that may be used in pose estimation
     *
     * @return the array of vision measurement
     */
    public VisionPoseMeasurement[] getVisionPoseMeasurements();

    /**
     * Returns the settings of the camera
     *
     * @return camera settings
     */
    public CameraSettings getCameraSettings();
  }

  /**
   * Update the specified camera and proccess it's inputs can vision measurment
   *
   * @param index the index of the camera to update
   */
  public void updateCamera(int index);

  /**
   * Add a camera to the subsytem
   *
   * @param camera the camera to add
   */
  public void addCamera(Camera camera);

  /**
   * Returns the cameras used by the subsystem
   *
   * @return thw list of cameras
   */
  public List<Camera> getCameras();

  /**
   * Returns the camera inputs of the cameras with in the subsystem
   *
   * @return a list of cameras inputs
   */
  public List<CameraInputsAutoLogged> getCameraInputs();

  /**
   * Returns the AprilTag field layout used by the subsystem
   *
   * @return the AprilTag field layout
   */
  public AprilTagFieldLayout getFieldLayout();
}
