package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.interfaces.Vision.Camera;
import java.util.Properties;

public class CameraBase implements Camera {
  public static CameraBase createCameraBase(Properties properties, String name) {

    Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.xMeters")),
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.yMeters")),
                Double.parseDouble(properties.getProperty(name + ".robotToCamera.zMeters"))),
            new Rotation3d(
                Double.parseDouble(properties.getProperty(name + "robotToCamera.rollDegrees")),
                Double.parseDouble(properties.getProperty(name + "robotToCamera.pitchDegrees")),
                Double.parseDouble(properties.getProperty(name + "robotToCamera.yawDegrees"))));

    CameraSettings settings = CameraSettings.getSettings(properties, name);

    return new CameraBase(name, robotToCamera, settings);
  }

  private final Transform3d robotToCamera;
  private final String name;
  private final CameraSettings settings;

  CameraBase(String name, Transform3d robotToCamera, CameraSettings settings) {
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.settings = settings;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  @Override
  public void update(CameraInputs inputs) {}

  @Override
  public VisionPoseMeasurement[] getVisionPoseMeasurements() {
    return new VisionPoseMeasurement[0];
  }

  @Override
  public CameraSettings getCameraSettings() {
    return settings;
  }
}
