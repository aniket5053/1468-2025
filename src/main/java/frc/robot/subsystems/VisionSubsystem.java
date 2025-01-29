package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  // Create PhotonCamera objects
  private final PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
  private final PhotonCamera rearCamera = new PhotonCamera("RearCamera");

  // Construct PhotonPoseEstimator
  private final Transform3d robotToFrontCamera =
      new Transform3d(
          new Translation3d(0.2176272, 0.0882396, 0.2332736),
          new Rotation3d(
              0,
              -0.294524311,
              -0.078644536)); // TODO: Change values when moving camera; 1/3/25: X: 8.568 in, Y:
  // 3.474 in, Z: 9.184 in, Yaw: -4.506 degrees, Pitch = -16.875
  // degrees, Roll: 0
  // private final Transform3d robotToAprilTagCamera = new Transform3d(new Translation3d(0.5, 0.0,
  // 0.5), new Rotation3d(0,0,0));     // EXAMPLE: Cam mounted facing forward, half a meter forward
  // of center, half a meter up from center

  private final Transform3d robotToRearCamera =
      new Transform3d( // turned camera 180
          new Translation3d(0.2330272, -0.0882396, 0.34671),
          new Rotation3d(0, -0.294524311, 3.14159265 - 0.078644536));
  private final PoseStrategy poseStrategy =
      PoseStrategy
          .MULTI_TAG_PNP_ON_COPROCESSOR; // TODO: Ensure that your camera is calibrated and 3D mode
  // is enabled. Read
  // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/apriltag-pipelines/multitag.html#multitag-localization
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2025Reefscape
          .loadAprilTagLayoutField(); // TODO: // The field from AprilTagFields will be different
  // depending on the game.

  PhotonPoseEstimator frontCamPoseEstimator =
      new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToFrontCamera);

  PhotonPoseEstimator rearCamPoseEstimator =
      new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToRearCamera);

  // Initialize variables (default if frontCameraTargetResults is empty)
  private boolean frontCameraHasTargets = false;
  private boolean multipleFrontCameraTargets = false;
  private String frontCameraAllIdsString = "No IDs";
  private int frontCameraBestTargetId =
      9999; // This indicates that something is wrong with the AprilTag camera
  private double frontCameraBestTargetYaw = 0;
  private double frontCameraBestTargetArea = 0;
  private double frontCameraBestTargetX = 0;
  private double frontCameraBestTargetY = 0;

  // Initialize variables (default if rearCameraTargetResults is empty)
  private boolean rearCameraHasTargets = false;
  private boolean multipleRearCameraTargets = false;
  private String rearCameaAllIdsString = "No IDs";
  private int rearCameraBestTargetId =
      9999; // This indicates that something is wrong with the AprilTag camera
  private double rearCameraBestTargetYaw = 0;
  private double rearCameraBestTargetArea = 0;
  private double rearCameraBestTargetX = 0;
  private double rearCameraBestTargetY = 0;

  private Matrix<N3, N1> curFrntCamStdDevs;
  private Matrix<N3, N1> curRearCamStdDevs;

  // TA TODO: RIght now have only 1 Single Tag StdDev, s/b OK, need to calibrate multitag StdDevs!!!
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> kMultiTagRearCamStdDevs = VecBuilder.fill(0.025, 0.025, .1);
  public static final Matrix<N3, N1> kMultiTagFrontCamStdDevs = VecBuilder.fill(0.025, 0.025, .1);
  // public static final Matrix<N3, N1> kMultiTagFrontCamStdDevs = VecBuilder.fill(0.1, 0.1, .4);

  public VisionSubsystem() {
    // Additional initialization if needed
  }

  @Override
  public void periodic() {

    /////////////////////////// Front Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// Front Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// Front Camera Processing
    // ////////////////////////////////////////////////

    // Read in relevant data from the Camera
    var frontCameraTargetResults = frontCamera.getAllUnreadResults();
    if (!frontCameraTargetResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var frontCameraResult = frontCameraTargetResults.get(frontCameraTargetResults.size() - 1);

      if (frontCameraResult.hasTargets()) {
        // At least one AprilTag was seen by the camera
        frontCameraHasTargets = true;

        frontCameraBestTargetX = frontCameraResult.getBestTarget().getBestCameraToTarget().getX();
        frontCameraBestTargetY = frontCameraResult.getBestTarget().getBestCameraToTarget().getY();

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> frontCameraTargets = frontCameraResult.getTargets();
        if (frontCameraTargets.size() > 1) {
          // Multiple AprilTags detected
          multipleFrontCameraTargets = true;
          frontCameraAllIdsString = "Mult IDs";
        } else {
          // Only one AprilTag detected
          multipleFrontCameraTargets = false;
          frontCameraAllIdsString = "One ID";
        }

        // Gets best target (for multiple targets)
        frontCameraBestTargetId = frontCameraResult.getBestTarget().getFiducialId();
        frontCameraBestTargetYaw = frontCameraResult.getBestTarget().getYaw();
        frontCameraBestTargetArea = frontCameraResult.getBestTarget().getArea();
      } else {
        // No AprilTags detected by camera
        frontCameraHasTargets = false;
        multipleFrontCameraTargets = false;
        frontCameraAllIdsString = "No IDs";
        frontCameraBestTargetId = 99;
        frontCameraBestTargetYaw = 0;
        frontCameraBestTargetArea = 0;
      }
    }
    // Publish to SmartDashboard
    SmartDashboard.putBoolean("FrntCam Targets?   ", frontCameraHasTargets);
    SmartDashboard.putBoolean("Mult FrntCam Trgts?", multipleFrontCameraTargets);
    //   SmartDashboard.putString("Mult FrntCam IDs", frontCameraAllIdsString);
    SmartDashboard.putNumber("BestFrntCam Tgt ID ", frontCameraBestTargetId);
    SmartDashboard.putNumber("BestFrntCam Tgt X  ", frontCameraBestTargetX);
    SmartDashboard.putNumber("BestFrntCam Tgt Y  ", frontCameraBestTargetY);
    SmartDashboard.putNumber("BestFrntCam Tgt Yaw", frontCameraBestTargetYaw);

    /////////////////////////// Rear Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// Rear Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// Rear Camera Processing
    // ////////////////////////////////////////////////

    // Read in relevant data from the Camera
    var rearCameraTargetResults = rearCamera.getAllUnreadResults();
    if (!rearCameraTargetResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var rearCameraResult = rearCameraTargetResults.get(rearCameraTargetResults.size() - 1);

      if (rearCameraResult.hasTargets()) {
        // At least one AprilTag was seen by the camera
        rearCameraHasTargets = true;

        rearCameraBestTargetX = rearCameraResult.getBestTarget().getBestCameraToTarget().getX();
        rearCameraBestTargetY = rearCameraResult.getBestTarget().getBestCameraToTarget().getY();

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> rearCameraTartgets = rearCameraResult.getTargets();
        if (rearCameraTartgets.size() > 1) {
          // Multiple AprilTags detected
          multipleRearCameraTargets = true;
          rearCameaAllIdsString = "Not finished yet oops";
        } else {
          // Only one AprilTag detected
          multipleRearCameraTargets = false;
          rearCameaAllIdsString = "Only one";
        }

        // Gets best target (for multiple targets)
        rearCameraBestTargetId = rearCameraResult.getBestTarget().getFiducialId();
        rearCameraBestTargetYaw = rearCameraResult.getBestTarget().getYaw();
        rearCameraBestTargetArea = rearCameraResult.getBestTarget().getArea();
      } else {
        // No Targets detected by camera
        rearCameraHasTargets = false;
        multipleRearCameraTargets = false;
        rearCameaAllIdsString = "No IDs";
        rearCameraBestTargetId = 99;
        rearCameraBestTargetYaw = 0;
        rearCameraBestTargetArea = 0;
      }
    }
    // Publish to SmartDashboard
    SmartDashboard.putBoolean("RearCam Targets?   ", rearCameraHasTargets);
    SmartDashboard.putBoolean("Mult RearCam Trgts?", multipleRearCameraTargets);
    //   SmartDashboard.putString("Multiple RearCam IDs", rearCameaAllIdsString);
    SmartDashboard.putNumber("BestRearCam Tgt ID ", rearCameraBestTargetId);
    SmartDashboard.putNumber("BestRearCam Tgt X  ", rearCameraBestTargetX);
    SmartDashboard.putNumber("BestRearCam Tgt Y  ", rearCameraBestTargetY);
    SmartDashboard.putNumber("BestRearCam Tgt Yaw", rearCameraBestTargetYaw);
  }

  //////////// Front Camera Getter methods to access target data
  public boolean frntCamTgtDectected() {
    return frontCameraHasTargets;
  }

  public double getFrntCamBestTgtId() {
    return frontCameraBestTargetId;
  }

  public double getFrntCamBestTgtX() {
    return frontCameraBestTargetX;
  }

  public double getFrntCamBestTgtY() {
    return frontCameraBestTargetY;
  }

  public double getFrntCamBestTgtYaw() {
    return frontCameraBestTargetYaw;
  }

  public double getFrntCamBestTgtArea() {
    return frontCameraBestTargetArea;
  }

  //////////// Rear Camera Getter methods to access target data
  public boolean rearCamTgtDectected() {
    return rearCameraHasTargets;
  }

  public double getRearCamBestTgtId() {
    return rearCameraBestTargetId;
  }

  public double getRearCamBestTgtX() {
    return rearCameraBestTargetX;
  }

  public double getRearCamBestTgtY() {
    return rearCameraBestTargetY;
  }

  public double getRearCamBestTgtYaw() {
    return rearCameraBestTargetYaw;
  }

  public double getRearCamBestTgtArea() {
    return rearCameraBestTargetArea;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseUsingFrntCamTgts() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var change : frontCamera.getAllUnreadResults()) {
      visionEst = frontCamPoseEstimator.update(change);
      updateEstimationFrntCamStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationFrntCamStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curFrntCamStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = frontCamPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curFrntCamStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagFrontCamStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curFrntCamStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationFrntCamStdDevs() {
    return curFrntCamStdDevs;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseUsingRearCamTgts() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : rearCamera.getAllUnreadResults()) {
      visionEst = rearCamPoseEstimator.update(change);
      updateEstimationRearCamStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationRearCamStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curRearCamStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = rearCamPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curRearCamStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagRearCamStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curRearCamStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationRearCamStdDevs() {
    return curRearCamStdDevs;
  }
}
