package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CameraInputsAutoLogged;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera.VisionPoseMeasurement;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase implements Vision {
  public class MatchingMeasurementInfo  {
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
  // for each camera, a map of the seen aprilTags to an list of poseMeasurements with that april tag
  // seen
  private List<Map<Integer, List<VisionPoseMeasurement>>> cameraTagPoses = new LinkedList<>();
  private AprilTagFieldLayout fieldLayout;
  private Optional<VisionMeasurementConsumer> visionMeasurementConsumer;

  // maximum allow time between 2 poseMeasurements to still be considered vaild, need to tune
  private final double TIMESTAMP_TOLERANCE_SECONDS = 10;
  // maximum distance between robot and tag for poseMeasurement to be vailid to be considered
  private final double MAXIMUM_SINGLE_TAG_DISTANCE_METERS = 2.0; // need to tune to robot
  // show if pose measurement is valid, reason (and matching pose if there is one) and
  // poseMeasurement data at
  // AdvagtageKit/RealOutputs/Vision/'cameraName'/PoseMeasurements/'poseIndex'/
  private final boolean VISION_LOGGING_DEBUG = true;

  public VisionSubsystem(
      AprilTagFieldLayout layout, Optional<VisionMeasurementConsumer> visionMeasurementConsumer) {
    fieldLayout = layout;
    cameras = new LinkedList<>();
    cameraInputs = new LinkedList<>();
    this.visionMeasurementConsumer = visionMeasurementConsumer;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.size(); i++) {
      updateCamera(i);
    }

    List<VisionPoseMeasurement> validPoseMeasurements = new LinkedList<>();
    Map<VisionPoseMeasurement, MatchingMeasurementInfo> measurmentToMatchMap = new HashMap<>();

    for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
      int i = -1;
      for (VisionPoseMeasurement poseMeasurement :
          cameras.get(cameraIndex).getVisionPoseMeasurements()) {
        i++;
        // if already valid no need to check
        if (validPoseMeasurements.contains(poseMeasurement)) {
          continue;
        }

        // more than one tag then valid for estimation
        if (poseMeasurement.targetIds.length >= 2) {
          validPoseMeasurements.add(poseMeasurement);
          continue;
        } else {

          // one tag seen but is close then validfor estimation
          if (poseMeasurement.robotToBestTargetDistanceInMeters != -1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= MAXIMUM_SINGLE_TAG_DISTANCE_METERS) {
            validPoseMeasurements.add(poseMeasurement);
            continue;
          }
          // check measurements from other cameras for matching tag
          else {
            // camera index, april tag id, pose measurement
            Optional<MatchingMeasurementInfo> measurementInfoOptional =
                getMatchingTagPoseMeasuremnet(cameraIndex, poseMeasurement);
            if (measurementInfoOptional.isPresent()) {
              VisionPoseMeasurement matchingPoseMeasurement =
                  measurementInfoOptional.get().measurement;

              validPoseMeasurements.add(poseMeasurement);
              measurmentToMatchMap.put(poseMeasurement, measurementInfoOptional.get());

              validPoseMeasurements.add(matchingPoseMeasurement);
              MatchingMeasurementInfo matchingMeasurementInfo = new MatchingMeasurementInfo(poseMeasurement, cameraIndex, measurementInfoOptional.get().aprilTag);
              measurmentToMatchMap.put(matchingPoseMeasurement, matchingMeasurementInfo);

              if (!validPoseMeasurements.contains(matchingPoseMeasurement)) {
                validPoseMeasurements.add(matchingPoseMeasurement);
              }
            }
          }
        }
      }
    }

    // debug info on measurements
    if (VISION_LOGGING_DEBUG) {
      for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
        for (int i = 0; i < cameras.get(cameraIndex).getVisionPoseMeasurements().length; i++) {
          VisionPoseMeasurement poseMeasurement =
              cameras.get(cameraIndex).getVisionPoseMeasurements()[i];
          boolean isValid = validPoseMeasurements.contains(poseMeasurement);
          String reason = "";
          String matchDebug = "N/A";

          if (poseMeasurement.targetIds.length >= 2) {
            reason = "MultiTag with IDs:" + Arrays.toString(poseMeasurement.targetIds);
          } else if (poseMeasurement.robotToBestTargetDistanceInMeters != -1
              && poseMeasurement.robotToBestTargetDistanceInMeters
                  <= MAXIMUM_SINGLE_TAG_DISTANCE_METERS) {
            reason =
                "Single tag with distance of : "
                    + poseMeasurement.robotToBestTargetDistanceInMeters;
          } else if (isValid) {
            reason = "Cross Tag Check,  Same AprilTag as different camera";
            MatchingMeasurementInfo matchInfo = measurmentToMatchMap.get(poseMeasurement);
            matchDebug = "Camera: "
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
    for (int i = 0; i < validPoseMeasurements.size(); i++) {
      double distanceMeters = validPoseMeasurements.get(i).robotToBestTargetDistanceInMeters;
      visionMeasurementConsumer
          .get()
          .add(
              validPoseMeasurements.get(i).robotPose,
              validPoseMeasurements.get(i).timestamp,
              VecBuilder.fill(distanceMeters / 2, distanceMeters / 2, distanceMeters / 2));
    }

    // cant clear all at the same time because camera are not sync
    // check if a pose has be in there for a lifespan based off of fps of data published by the
    // camera and remove if its lifespan has passed
    double currentTime = Timer.getFPGATimestamp();
    for (int cameraIndex = 0; cameraIndex < cameraTagPoses.size(); cameraIndex++) {
      // the time between one publish and the next
      double oneMeasureDurationSeconds = 1.0 / cameras.get(cameraIndex).getCameraSettings().fps;

      double measureLifespan = oneMeasureDurationSeconds * 5; // found 5 from testing in sim

      Map<Integer, List<VisionPoseMeasurement>> tagMap = cameraTagPoses.get(cameraIndex);

      for (List<VisionPoseMeasurement> posesAtSeenTag : tagMap.values()) {
        posesAtSeenTag.removeIf(
            measurement -> currentTime - measurement.timestamp > measureLifespan);
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
            List<VisionPoseMeasurement> poseMeasurementList =
                new ArrayList<VisionPoseMeasurement>();
            poseMeasurementList.add(poseMeasurements[i]);
            cameraTagPoses
                .get(cameraIndex)
                .put(poseMeasurements[i].targetIds[j], poseMeasurementList);
          }
        }
        // debug infomation on measurements
        if (VISION_LOGGING_DEBUG) {
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

  // returns optional of pair (camera index, pair (aprilTag, poseMe)) or empty optional for no match
  private Optional<MatchingMeasurementInfo>
      getMatchingTagPoseMeasuremnet(int poseCameraIndex, VisionPoseMeasurement poseMeasurement) {
    for (int knownTagsCameraIndex = 0;
        knownTagsCameraIndex < cameraTagPoses.size();
        knownTagsCameraIndex++) {
      // Dont check the same camera as the poseMeasurement
      if (poseCameraIndex == knownTagsCameraIndex) {
        continue;
      }
      // map of the april tag ids to the poseMeasurements that saw the tag
      Map<Integer, List<VisionPoseMeasurement>> seenTagIdmap =
          cameraTagPoses.get(knownTagsCameraIndex);

      for (int aprilTagId : poseMeasurement.targetIds) {
        // if the aprilTagId is in the map
        if (!seenTagIdmap.containsKey(aprilTagId)) {
          continue;
        }

        List<VisionPoseMeasurement> posesAtSeenTag = seenTagIdmap.get(aprilTagId);
        for (int i = 0; i < posesAtSeenTag.size(); i++) {
          VisionPoseMeasurement possiblePoseMeasurementMatch = posesAtSeenTag.get(i); // get latest

          // if the measurements where at different times, then dont comapare
          if (Math.abs(poseMeasurement.timestamp - possiblePoseMeasurementMatch.timestamp)
              > TIMESTAMP_TOLERANCE_SECONDS) {
            continue;
          }
          MatchingMeasurementInfo info = new MatchingMeasurementInfo(possiblePoseMeasurementMatch, knownTagsCameraIndex, aprilTagId);
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
