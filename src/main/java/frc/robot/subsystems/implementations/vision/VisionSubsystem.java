package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CameraInputsAutoLogged;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera.VisionPoseMeasurement;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase implements Vision {
  public class MatchingMeasurementInfo {
    public VisionPoseMeasurement measurement;
    public int cameraIndex;
    public int aprilTag;

    MatchingMeasurementInfo(VisionPoseMeasurement measurement, int cameraIndex, int aprilTag) {
      this.measurement = measurement;
      this.cameraIndex = cameraIndex;
      this.aprilTag = aprilTag;
    }
  }

  private List<Camera> cameras;
  private List<CameraInputsAutoLogged> cameraInputs;
  // for each camera, a map of the seen aprilTags to an set of poseMeasurements with that april tag
  // seen
  private List<Map<Integer, Set<VisionPoseMeasurement>>> cameraTagPoses = new LinkedList<>();
  private AprilTagFieldLayout fieldLayout;
  private Optional<VisionMeasurementConsumer> visionMeasurementConsumer;

  private VisionSettings settings;

  public VisionSubsystem(
      AprilTagFieldLayout layout,
      Optional<VisionMeasurementConsumer> visionMeasurementConsumer,
      VisionSettings settings) {
    fieldLayout = layout;
    cameras = new LinkedList<>();
    cameraInputs = new LinkedList<>();
    this.visionMeasurementConsumer = visionMeasurementConsumer;
    this.settings = settings;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.size(); i++) {
      updateCamera(i);
    }

    Set<VisionPoseMeasurement> validPoseMeasurements = new HashSet<VisionPoseMeasurement>();
    Map<VisionPoseMeasurement, MatchingMeasurementInfo> measurementToMatchMap = new HashMap<>();

    for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
      for (VisionPoseMeasurement poseMeasurement :
          cameras.get(cameraIndex).getVisionPoseMeasurements()) {

        // more than one tag then then the measurement is valid for estimation
        if (poseMeasurement.targetIds.length >= 2
            && poseMeasurement.robotToBestTargetDistanceInMeters
                <= settings.maximumMultiTagDistanceMeters) {
          validPoseMeasurements.add(poseMeasurement);
          continue;
        } else {

          // one tag seen and is close than max distance then measurement is valid for estimation
          if (poseMeasurement.targetIds.length == 1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= settings.maximumSingleTagDistanceMeters || (poseMeasurement.targetIds.length == 1
              && DriverStation.isAutonomousEnabled() && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= 2.6)) {
            validPoseMeasurements.add(poseMeasurement);
            continue;
          }
          // check measurements from other cameras for matching tag
          else {
            Optional<MatchingMeasurementInfo> measurementInfoOptional =
                getMatchingTagPoseMeasurement(cameraIndex, poseMeasurement);

            if (measurementInfoOptional.isPresent()) {
              VisionPoseMeasurement matchingPoseMeasurement =
                  measurementInfoOptional.get().measurement;

              validPoseMeasurements.add(poseMeasurement);
              measurementToMatchMap.put(poseMeasurement, measurementInfoOptional.get());

              validPoseMeasurements.add(matchingPoseMeasurement);
              MatchingMeasurementInfo matchingMeasurementInfo =
                  new MatchingMeasurementInfo(
                      poseMeasurement, cameraIndex, measurementInfoOptional.get().aprilTag);
              measurementToMatchMap.put(matchingPoseMeasurement, matchingMeasurementInfo);
            }
          }
        }
      }
    }

    // debug info on measurements
    if (settings.visionLoggingDebug) {
      for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
        for (int i = 0; i < cameras.get(cameraIndex).getVisionPoseMeasurements().length; i++) {
          VisionPoseMeasurement poseMeasurement =
              cameras.get(cameraIndex).getVisionPoseMeasurements()[i];
          boolean isValid = validPoseMeasurements.contains(poseMeasurement);
          String reason = "";
          String matchDebug = "N/A";

          if (poseMeasurement.targetIds.length >= 2
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= settings.maximumMultiTagDistanceMeters) {
            reason = "MultiTag with IDs:" + Arrays.toString(poseMeasurement.targetIds);
          } else if (poseMeasurement.targetIds.length == 1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= settings.maximumSingleTagDistanceMeters) {
            reason =
                "Single tag with distance of : "
                    + poseMeasurement.robotToBestTargetDistanceInMeters;
          } else if (isValid) {
            reason = "Cross Tag Check,  Same AprilTag as different camera";
            MatchingMeasurementInfo matchInfo = measurementToMatchMap.get(poseMeasurement);
            if(matchInfo == null) {
              break;
              
            }
            matchDebug =
                "Camera: "
                    + cameras.get(matchInfo.cameraIndex).getName()
                    + " MatchingTagId: "
                    + matchInfo.aprilTag
                    + " timestamp: "
                    + matchInfo.measurement.timestamp;

          } else {
            reason =
                "Failed to have Single Tag with a match and is greater than min valid distance: "
                    + poseMeasurement.robotToBestTargetDistanceInMeters;
          }

          Logger.recordOutput(
              "Vision/"
                  + cameras.get(cameraIndex).getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/reason",
              reason);
          Logger.recordOutput(
              "Vision/"
                  + cameras.get(cameraIndex).getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/isValid",
              isValid);
          Logger.recordOutput(
              "Vision/"
                  + cameras.get(cameraIndex).getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/matchingMeasurementInfo",
              matchDebug);
        }
      }
    }

    // add valid pose measurment to consumer
    for (VisionPoseMeasurement measurement : validPoseMeasurements) {
      double distanceMeters = measurement.robotToBestTargetDistanceInMeters;
      visionMeasurementConsumer
          .get()
          .add(
              measurement.robotPose,
              measurement.timestamp,
              VecBuilder.fill(distanceMeters / 2, distanceMeters / 2, distanceMeters / 2));
    }

    // cant clear all at the same time because camera are not sync
    // check if a pose has been there for lifespan based off of fps of data published by the
    // camera/processor and remove if its lifespan has passed
    double currentTime = Timer.getFPGATimestamp();
    for (int cameraIndex = 0; cameraIndex < cameraTagPoses.size(); cameraIndex++) {

      // tuned by hand to see when new data from photonvision is available. by seeing how log it
      // takes for a photonvision result.
      double measurementLifespan = 0.25;

      Map<Integer, Set<VisionPoseMeasurement>> tagMap = cameraTagPoses.get(cameraIndex);

      for (Set<VisionPoseMeasurement> posesAtSeenTag : tagMap.values()) {
        posesAtSeenTag.removeIf(
            measurement -> currentTime - measurement.timestamp > measurementLifespan);
      }
    }
  }

  @Override
  public void updateCamera(int cameraIndex) {
    Camera camera = cameras.get(cameraIndex);
    CameraInputsAutoLogged inputs = cameraInputs.get(cameraIndex);

    // update  and publish inputs
    Logger.processInputs("Vision/" + camera.getName(), inputs);
    camera.update(inputs);

    if (visionMeasurementConsumer.isPresent()) {

      VisionPoseMeasurement[] poseMeasurements = camera.getVisionPoseMeasurements();

      // add map measurments to tags in cameraTagPoses for camera
      for (int i = 0; i < poseMeasurements.length; i++) {
        for (int j = 0; j < poseMeasurements[i].targetIds.length; j++) {
          if (cameraTagPoses.get(cameraIndex).containsKey(poseMeasurements[i].targetIds[j])) {
            cameraTagPoses
                .get(cameraIndex)
                .get(poseMeasurements[i].targetIds[j])
                .add(poseMeasurements[i]);
          } else {
            // make new key if first time tag is being seen
            Set<VisionPoseMeasurement> poseMeasurementList = new HashSet<VisionPoseMeasurement>();
            poseMeasurementList.add(poseMeasurements[i]);
            cameraTagPoses
                .get(cameraIndex)
                .put(poseMeasurements[i].targetIds[j], poseMeasurementList);
          }
        }
        // debug information on measurements
        if (settings.visionLoggingDebug) {
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/timestamp",
              poseMeasurements[i].timestamp);

          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/targetIds",
              poseMeasurements[i].targetIds);
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/robotPose",
              poseMeasurements[i].robotPose);
          Logger.recordOutput(
              "Vision/"
                  + camera.getName()
                  + "/PoseMeasurements/"
                  + String.valueOf(i)
                  + "/bestTargetDistance",
              poseMeasurements[i].robotToBestTargetDistanceInMeters);
        }
      }
    }
  }

  /**
   * Returns an Optional based on the given camera and pose measurement.
   *
   * <p>The Optional contains the matching measurment infomation if there is a measurment from a
   * different camera, within the timestamp tolerance, that has same AprilTag ID detected as the
   * given measurment.
   *
   * @param poseCameraIndex the index of the camera that the given pose measurment is from
   * @param poseMeasurement the vision pose measurement
   * @return an empty Optional if there is no pose measurment, or an Optional holding the matching
   *     pose measurement info
   */
  private Optional<MatchingMeasurementInfo> getMatchingTagPoseMeasurement(
      int poseCameraIndex, VisionPoseMeasurement poseMeasurement) {
    for (int knownTagsCameraIndex = 0;
        knownTagsCameraIndex < cameraTagPoses.size();
        knownTagsCameraIndex++) {
      // Dont check the same camera as the poseMeasurement
      if (poseCameraIndex == knownTagsCameraIndex) {
        continue;
      }
      // map of the april tag ids to the poseMeasurements that saw the tag
      Map<Integer, Set<VisionPoseMeasurement>> seenTagIdmap =
          cameraTagPoses.get(knownTagsCameraIndex);

      for (int aprilTagId : poseMeasurement.targetIds) {
        // if the aprilTagId is in the map
        if (!seenTagIdmap.containsKey(aprilTagId)) {
          continue;
        }

        Set<VisionPoseMeasurement> posesAtSeenTag = seenTagIdmap.get(aprilTagId);
        for (VisionPoseMeasurement possiblePoseMeasurementMatch : posesAtSeenTag) {
          // if the measurements are at different times, then dont comapare
          if (Math.abs(poseMeasurement.timestamp - possiblePoseMeasurementMatch.timestamp)
              > settings.timestampToleranceSeconds) {
            continue;
          }
          MatchingMeasurementInfo info =
              new MatchingMeasurementInfo(
                  possiblePoseMeasurementMatch, knownTagsCameraIndex, aprilTagId);
          return Optional.of(info);
        }
      }
    }

    return Optional.empty();
  }

  @Override
  public void addCamera(Camera camera) {
    cameras.add(camera);
    cameraInputs.add(new CameraInputsAutoLogged());
    cameraTagPoses.add(new HashMap<>());
  }

  @Override
  public List<Camera> getCameras() {
    return cameras;
  }

  @Override
  public List<CameraInputsAutoLogged> getCameraInputs() {
    return cameraInputs;
  }

  @Override
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }
}
