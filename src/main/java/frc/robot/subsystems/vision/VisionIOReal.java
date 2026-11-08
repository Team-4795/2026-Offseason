package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;

public class VisionIOReal implements VisionIO {
  PhotonCamera camera;
  PhotonPoseEstimator estimator;
  PhotonTargetSortMode sortMode;

  public VisionIOReal(int camIndex) {
    camera = new PhotonCamera(VisionConstants.CAM_NAMES[camIndex]);

    estimator =
        new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.CAM_POSES[camIndex]);

    sortMode = PhotonTargetSortMode.Largest;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      visionEst = estimator.estimateCoprocMultiTagPose(result); // Multitag estimation for global pose
      inputs.latency = result.metadata.getLatencyMillis();

      // Singletag if only one tag is in view
      // or if were scoring/intaking
      // (i mean all of our cameras are on the front so its effectively singletag anyway)
      // TODO better fine pose estimation? like whatever 6328 was cooking up
      if (visionEst.isEmpty() || VisionConstants.localMode) {
        visionEst = estimator.estimateLowestAmbiguityPose(result);
      }

      // Update logs if we're getting vision data
      visionEst.ifPresentOrElse(
          (pose) -> {
            inputs.pose = new Pose3d[] {pose.estimatedPose};
            inputs.timestamp = new double[] {pose.timestampSeconds};
            inputs.tags =
                pose.targetsUsed.stream()
                    .mapToInt(
                        (target) -> {
                          return target.getFiducialId();
                        })
                    .toArray();
          },
          () -> {
            inputs.pose = new Pose3d[] {};
            inputs.timestamp = new double[] {};
            inputs.tags = new int[] {};
          });
    }
  }
}
