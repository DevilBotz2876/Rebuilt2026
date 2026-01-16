package frc.robot.subsystems.implementations.vision.camera;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.interfaces.Vision.Camera;

public class CameraBase implements Camera {
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
