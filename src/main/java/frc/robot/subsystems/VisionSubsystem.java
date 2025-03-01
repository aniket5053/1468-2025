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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final PhotonCamera rightFrtCamera = new PhotonCamera("rightFrtCamera");
  private final PhotonCamera leftFrtCamera = new PhotonCamera("leftFrtCamera");

  // Construct PhotonPoseEstimator

  // The two cameras have the same mount and are mounted up rightFrt facing forward offsets are:
  // rightFrt Left Cam:  (x,y,z) = (12.8343, +9.9354, 11.039) inches
  // rightFrt Right Cam: (x,y,z) = (12.8343, -9.9354, 11.039) inches
  // rightFrt Left Cam:  (R,P,Y) = (-5.0, -20.1, -10.0 ) degrees
  // rightFrt Right Cam: (R,P,Y) = (-5.0, -20.1, +10.0 ) degrees

  private final Transform3d robotToRightFrtCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.8343), // verified correct
              Units.inchesToMeters(-9.9354), // verified negative is correct
              Units.inchesToMeters(-11.039)), // verified correct
          new Rotation3d(
              Units.degreesToRadians(+5.0), // was -
              Units.degreesToRadians(-20.1), // verified negative is correct -20.1
              Units.degreesToRadians(-10.5))); // verified negative is correct

  private final Transform3d robotToLeftFrtCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.8343), // verified correct
              Units.inchesToMeters(9.9354), // verified correct
              Units.inchesToMeters(-11.039)), // verified correct
          new Rotation3d(
              Units.degreesToRadians(-5.0), // was -
              Units.degreesToRadians(-15.1), // verified negative is correct  -15.1
              Units.degreesToRadians(+10.5))); // verified positive is correct

  // Old Camera setup //////////////////////////////////////////////////////
  /*
    private final Transform3d robotToFrontCamera =
        new Transform3d(
            new Translation3d(0.2176272, 0.0882396, 0.2332736),
            new Rotation3d(
                0,  -0.294524311, -0.078644536));
    // 3.474 in, Z: 9.184 in, Yaw: -4.506 degrees, Pitch = -16.875
    // degrees, Roll: 0
    // private final Transform3d robotToAprilTagCamera = new Transform3d(new Translation3d(0.5, 0.0,
    // 0.5), new Rotation3d(0,0,0));     // EXAMPLE: Cam mounted facing forward, half a meter forward
    // of center, half a meter up from center

    private final Transform3d robotToRearCamera =
        new Transform3d( // turned camera 180
            new Translation3d(0.2330272, -0.0882396, 0.34671),
            new Rotation3d(0, -0.294524311, 3.14159265 - 0.078644536));
  */
  // Old Camera setup //////////////////////////////////////////////////////

  private final PoseStrategy poseStrategy =
      PoseStrategy
          .MULTI_TAG_PNP_ON_COPROCESSOR; // TODO: Ensure that your camera is calibrated and 3D mode
  // is enabled. Read
  // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/apriltag-pipelines/multitag.html#multitag-localization
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(); // TODO: // The field from
  // AprilTagFields will be different
  // depending on the game.

  PhotonPoseEstimator rightFrtCamPoseEstimator =
      new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToRightFrtCamera);

  PhotonPoseEstimator leftFrtCamPoseEstimator =
      new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToLeftFrtCamera);

  // Initialize variables (default if rightFrtCameraTargetResults is empty)
  private boolean rightFrtCameraHasTargets = false;
  private boolean multiplerightFrtCameraTargets = false;
  //  private String rightFrtCameraAllIdsString = "No IDs";
  private int rightFrtCameraBestTargetId =
      9999; // This indicates that something is wrong with the AprilTag camera
  private double rightFrtCameraBestTargetYaw = 0;
  private double rightFrtCameraBestTargetArea = 0;
  private double rightFrtCameraBestTargetX = 0;
  private double rightFrtCameraBestTargetY = 0;

  // Initialize variables (default if leftFrtCameraTargetResults is empty)
  private boolean leftFrtCameraHasTargets = false;
  private boolean multipleleftFrtCameraTargets = false;
  //  private String leftFrtCameaAllIdsString = "No IDs";
  private int leftFrtCameraBestTargetId =
      9999; // This indicates that something is wrong with the AprilTag camera
  private double leftFrtCameraBestTargetYaw = 0;
  private double leftFrtCameraBestTargetArea = 0;
  private double leftFrtCameraBestTargetX = 0;
  private double leftFrtCameraBestTargetY = 0;

  double TgtID = 99.0;
  // some variables to override photonVision "Best" with our own "best" (based on area only)
  double ltCamTgt0Area = 0.0;
  double ltCamTgt1Area = 0.0;
  double rtCamTgt0Area = 0.0;
  double rtCamTgt1Area = 0.0;

  private Matrix<N3, N1> currightFrtCamStdDevs;
  private Matrix<N3, N1> curleftFrtCamStdDevs;

  // TA TODO: RIght now have only 1 Single Tag StdDev, s/b OK, need to calibrate multitag StdDevs!!!
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> kMultiTagleftFrtCamStdDevs = VecBuilder.fill(0.025, 0.025, .1);
  public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs =
      VecBuilder.fill(0.025, 0.025, .1);
  // public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs = VecBuilder.fill(0.1, 0.1, .4);

  public VisionSubsystem() {
    // Additional initialization if needed
  }

  @Override
  public void periodic() {

    /////////////////////////// rightFrt Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// rightFrt Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// rightFrt Camera Processing
    // ////////////////////////////////////////////////

    // Read in relevant data from the Camera
    var rightFrtCameraTargetResults = rightFrtCamera.getAllUnreadResults();
    if (!rightFrtCameraTargetResults.isEmpty()) {

      // Camera processed a new frame since last
      // Get the last one in the list.
      var rightFrtCameraResult =
          rightFrtCameraTargetResults.get(rightFrtCameraTargetResults.size() - 1);

      if (rightFrtCameraResult.hasTargets()) {

        // At least one AprilTag was seen by the camera
        rightFrtCameraHasTargets = true;

        // Gets best target data
        rightFrtCameraBestTargetId = rightFrtCameraResult.getBestTarget().getFiducialId();
        rightFrtCameraBestTargetYaw = rightFrtCameraResult.getBestTarget().getYaw();
        rightFrtCameraBestTargetArea = rightFrtCameraResult.getBestTarget().getArea();
        rightFrtCameraBestTargetX =
            rightFrtCameraResult.getBestTarget().getBestCameraToTarget().getX();
        rightFrtCameraBestTargetY =
            rightFrtCameraResult.getBestTarget().getBestCameraToTarget().getY();

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> rightFrtCameraTargets = rightFrtCameraResult.getTargets();
        if (rightFrtCameraTargets.size() > 1) {
          // Multiple AprilTags detected

          multiplerightFrtCameraTargets = true;
          //         rightFrtCameraAllIdsString = "Mult IDs";

          // TA TODO: My attempt at determining best target since photonvisions algo sucks (must
          // test)
          rtCamTgt0Area = rightFrtCameraTargets.get(0).getArea();
          rtCamTgt1Area = rightFrtCameraTargets.get(1).getArea();
          if (rtCamTgt0Area >= rtCamTgt1Area) {
            leftFrtCameraBestTargetId = rightFrtCameraTargets.get(0).getFiducialId();
            leftFrtCameraBestTargetArea = rtCamTgt0Area;
          } else {
            leftFrtCameraBestTargetId = rightFrtCameraTargets.get(1).getFiducialId();
            leftFrtCameraBestTargetArea = rtCamTgt1Area;
          }

        } else {
          // Only one AprilTag detected

          multiplerightFrtCameraTargets = false;
          //         rightFrtCameraAllIdsString = "One ID";
        }

      } else {

        // No AprilTags detected by camera
        rightFrtCameraHasTargets = false;
        multiplerightFrtCameraTargets = false;
        //       rightFrtCameraAllIdsString = "No IDs";
        rightFrtCameraBestTargetId = 99;
        rightFrtCameraBestTargetYaw = 0;
        rightFrtCameraBestTargetArea = 0;
      }
    }

    /////////////////////////// leftFrt Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// leftFrt Camera Processing
    // ////////////////////////////////////////////////
    /////////////////////////// leftFrt Camera Processing
    // ////////////////////////////////////////////////

    // Read in relevant data from the Camera
    var leftFrtCameraTargetResults = leftFrtCamera.getAllUnreadResults();
    if (!leftFrtCameraTargetResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var leftFrtCameraResult =
          leftFrtCameraTargetResults.get(leftFrtCameraTargetResults.size() - 1);

      if (leftFrtCameraResult.hasTargets()) {
        // At least one AprilTag was seen by the camera
        leftFrtCameraHasTargets = true;
        // Get best target data
        leftFrtCameraBestTargetId = leftFrtCameraResult.getBestTarget().getFiducialId();
        leftFrtCameraBestTargetYaw = leftFrtCameraResult.getBestTarget().getYaw();
        leftFrtCameraBestTargetArea = leftFrtCameraResult.getBestTarget().getArea();
        leftFrtCameraBestTargetX =
            leftFrtCameraResult.getBestTarget().getBestCameraToTarget().getX();
        leftFrtCameraBestTargetY =
            leftFrtCameraResult.getBestTarget().getBestCameraToTarget().getY();

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> leftFrtCameraTartgets = leftFrtCameraResult.getTargets();
        if (leftFrtCameraTartgets.size() > 1) {
          // Multiple AprilTags detected
          multipleleftFrtCameraTargets = true;

          // TA TODO: My attempt at determining best target since photonvisions algo sucks (must
          // test)
          ltCamTgt0Area = leftFrtCameraTartgets.get(0).getArea();
          ltCamTgt1Area = leftFrtCameraTartgets.get(1).getArea();
          if (ltCamTgt0Area >= ltCamTgt1Area) {
            leftFrtCameraBestTargetId = leftFrtCameraTartgets.get(0).getFiducialId();
            leftFrtCameraBestTargetArea = ltCamTgt0Area;
          } else {
            leftFrtCameraBestTargetId = leftFrtCameraTartgets.get(1).getFiducialId();
            leftFrtCameraBestTargetArea = ltCamTgt1Area;
          }

          //          leftFrtCameaAllIdsString = "Not finished yet oops";
        } else {
          // Only one AprilTag detected
          multipleleftFrtCameraTargets = false;
          //          leftFrtCameaAllIdsString = "Only one";
        }

        // Get best target data
        leftFrtCameraBestTargetId = leftFrtCameraResult.getBestTarget().getFiducialId();
        leftFrtCameraBestTargetYaw = leftFrtCameraResult.getBestTarget().getYaw();
        leftFrtCameraBestTargetArea = leftFrtCameraResult.getBestTarget().getArea();
      } else {
        // No Targets detected by camera
        leftFrtCameraHasTargets = false;
        multipleleftFrtCameraTargets = false;
        //        leftFrtCameaAllIdsString = "No IDs";
        leftFrtCameraBestTargetId = 99;
        leftFrtCameraBestTargetYaw = 0;
        leftFrtCameraBestTargetArea = 0;
      }
    }

    determineAprilTagToDriveTo();

    // Publish to SmartDashboard

    SmartDashboard.putNumber("DriveTo AprilTag", TgtID);

    SmartDashboard.putBoolean("RightFrtCam Targets?   ", rightFrtCameraHasTargets);
    SmartDashboard.putBoolean("Mult RightFrtCam Trgts?", multiplerightFrtCameraTargets);
    //   SmartDashboard.putString("Mult rightFrtCam IDs", rightFrtCameraAllIdsString);
    SmartDashboard.putNumber("BestRightFrtCam Tgt ID ", rightFrtCameraBestTargetId);
    SmartDashboard.putNumber("BestRightFrtCam Tgt Area", rightFrtCameraBestTargetArea);

    SmartDashboard.putBoolean("leftFrtCam Targets?   ", leftFrtCameraHasTargets);
    SmartDashboard.putBoolean("Mult leftFrtCam Trgts?", multipleleftFrtCameraTargets);
    //   SmartDashboard.putString("Multiple leftFrtCam IDs", leftFrtCameaAllIdsString);
    SmartDashboard.putNumber("BestleftFrtCam Tgt ID ", leftFrtCameraBestTargetId);
    SmartDashboard.putNumber("BestleftFrtCam Tgt Area", leftFrtCameraBestTargetArea);
  }

  //////////// rightFrt Camera Getter methods to access target data
  public boolean rightFrtCamTgtDectected() {
    return rightFrtCameraHasTargets;
  }

  public double getrightFrtCamBestTgtId() {
    return rightFrtCameraBestTargetId;
  }

  public double getrightFrtCamBestTgtX() {
    return rightFrtCameraBestTargetX;
  }

  public double getrightFrtCamBestTgtY() {
    return rightFrtCameraBestTargetY;
  }

  public double getrightFrtCamBestTgtYaw() {
    return rightFrtCameraBestTargetYaw;
  }

  public double getrightFrtCamBestTgtArea() {
    return rightFrtCameraBestTargetArea;
  }

  //////////// leftFrt Camera Getter methods to access target data
  public boolean leftFrtCamTgtDectected() {
    return leftFrtCameraHasTargets;
  }

  public double getleftFrtCamBestTgtId() {
    return leftFrtCameraBestTargetId;
  }

  public double getleftFrtCamBestTgtX() {
    return leftFrtCameraBestTargetX;
  }

  public double getleftFrtCamBestTgtY() {
    return leftFrtCameraBestTargetY;
  }

  public double getleftFrtCamBestTgtYaw() {
    return leftFrtCameraBestTargetYaw;
  }

  public double getleftFrtCamBestTgtArea() {
    return leftFrtCameraBestTargetArea;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseUsingrightFrtCamTgts() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var change : rightFrtCamera.getAllUnreadResults()) {
      visionEst = rightFrtCamPoseEstimator.update(change);
      updateEstimationrightFrtCamStdDevs(visionEst, change.getTargets());
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
  private void updateEstimationrightFrtCamStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      currightFrtCamStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = rightFrtCamPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        currightFrtCamStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagrightFrtCamStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        currightFrtCamStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationrightFrtCamStdDevs() {
    return currightFrtCamStdDevs;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseUsingleftFrtCamTgts() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : leftFrtCamera.getAllUnreadResults()) {
      visionEst = leftFrtCamPoseEstimator.update(change);
      updateEstimationleftFrtCamStdDevs(visionEst, change.getTargets());
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
  private void updateEstimationleftFrtCamStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curleftFrtCamStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = leftFrtCamPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        curleftFrtCamStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagleftFrtCamStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curleftFrtCamStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationleftFrtCamStdDevs() {
    return curleftFrtCamStdDevs;
  }

  public void determineAprilTagToDriveTo() {

    TgtID = 99.0;
    double leftCamBestId = 99.0;
    double rightCamBestId = 99.0;
    double leftCamBestIdArea = 0.0;
    double rightCamBestIdArea = 0.0;

    if (rightFrtCamTgtDectected()
        && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red)
                && (getrightFrtCamBestTgtId() >= 6)
                && (getrightFrtCamBestTgtId() <= 11))
            || (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Blue)
                && (getrightFrtCamBestTgtId() >= 17)
                && (getrightFrtCamBestTgtId() <= 22)))) {

      rightCamBestId = getrightFrtCamBestTgtId();
      rightCamBestIdArea = getrightFrtCamBestTgtArea();
    }

    if (leftFrtCamTgtDectected()
        && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red)
                && (getleftFrtCamBestTgtId() >= 6)
                && (getleftFrtCamBestTgtId() <= 11))
            || (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Blue)
                && (getleftFrtCamBestTgtId() >= 17)
                && (getleftFrtCamBestTgtId() <= 22)))) {

      leftCamBestId = getleftFrtCamBestTgtId();
      leftCamBestIdArea = getleftFrtCamBestTgtArea();
    }

    if (leftCamBestIdArea >= rightCamBestIdArea) TgtID = leftCamBestId;
    else TgtID = rightCamBestId;
  }

  public double getTrgtIdToDriveTo() {
    return TgtID;
  }
}
