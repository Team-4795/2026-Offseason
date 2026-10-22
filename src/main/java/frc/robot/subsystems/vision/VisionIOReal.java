package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;

public class VisionIOReal implements VisionIO {
  PhotonCamera camera;
  PhotonPoseEstimator estimator;
  PhotonTargetSortMode sortMode;

  public VisionIOReal(int camIndex) {
    camera = new PhotonCamera(VisionConstants.CAM_NAMES[camIndex]);

    estimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.CAM_POSES[camIndex]);

    sortMode = PhotonTargetSortMode.Largest;
  }
}
