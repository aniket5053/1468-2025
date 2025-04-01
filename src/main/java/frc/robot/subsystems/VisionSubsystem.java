package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  // Construct PhotonPoseEstimator

  // The two cameras have the same mount and are mounted up rightFrt facing forward offsets are:
  // rightFrt Left Cam:  (x,y,z) = (12.8343, +9.9354, 11.039) inches
  // rightFrt Right Cam: (x,y,z) = (12.8343, -9.9354, 11.039) inches
  // rightFrt Left Cam:  (R,P,Y) = (-5.0, -20.1, -10.0 ) degrees
  // rightFrt Right Cam: (R,P,Y) = (-5.0, -20.1, +10.0 ) degrees

  // Original XYZ for Right Camera (12.8343, -9.9354, -11.039) - these numbers are slightly off
  // vector from what's in CAD but close - SZ
  // private final double rightX = 12.8439; // inch
  // private final double rightY = -9.9281;
  // private final double rightZ = -11.0344;

  // // Original XYZ for Left Camera (12.8343, 9.9354, -11.039) - these numbers are slightly off
  // vector
  // // from what's in CAD but close - SZ
  // private final double leftX = 12.8439; // inch
  // private final double leftY = 9.9281;
  // private final double leftZ = -11.0344;

  // // Normal unit vector orthogonal to camera YZ plane/x-axis pointing towards center of robot
  // private final double leftNormalX = -0.9517; // inch
  // private final double leftNormalY = -0.1711;
  // private final double leftNormalZ = -0.255; // was +

  // private final double rightNormalX = -0.9517; // inch
  // private final double rightNormalY = 0.1711;
  // private final double rightNormalZ = -0.255; // was +
  // // TA TODO: Must optimize - started with 3.0, did not reach reef, 2.5 was very inaccurate, but
  // // reached
  // private final double scalingFactor = 0; // was 2.75 to find camera focus position

  // // Calculates new camera positions based on scaling factor

  // private final double adjustedRightX = rightX + (scalingFactor * rightNormalX);
  // private final double adjustedRightY = rightY + (scalingFactor * rightNormalY);
  // private final double adjustedRightZ = rightZ + (scalingFactor * rightNormalZ);

  // private final double adjustedLeftX = leftX + (scalingFactor * leftNormalX);
  // private final double adjustedLeftY = leftY + (scalingFactor * leftNormalY);
  // private final double adjustedLeftZ = leftZ + (scalingFactor * leftNormalZ);

  // Original Camera Mount used at hofstra //////////////////////////////////////////////////
  // Original Camera Mount used at hofstra //////////////////////////////////////////////////
  // Original Camera Mount used at hofstra //////////////////////////////////////////////////

  // private final double cameraOffsetX = 12.8439; // CAD looks correct
  // private final double cameraOffsetY =
  //     9.2; // by measurement to camera focal plane - ruler measurement was 9.9425
  // private final double cameraOffsetZ =
  //     11.0344; // by measurement to camera focal plane - ruler measurement

  // private final double rtCameraOffsetRoll = 6.1;
  // private final double rtCameraOffsetPitch = -14.3;
  // private final double rtCameraOffesetYaw = -7.45; // was 8.5 had about a 1.2 d error

  // private final double ltCameraOffsetRoll = -5.5;
  // private final double ltCameraOffsetPitch = -15.3;
  // private final double ltCameraOffesetYaw = +9.7; // was 8.5

  // End Original Camera Mount used at hofstra //////////////////////////////////////////////////
  // End Original Camera Mount used at hofstra //////////////////////////////////////////////////
  // End Original Camera Mount used at hofstra //////////////////////////////////////////////////

  // Ofsets for Camera mounts with Yaw only ////////////////////////////////////////////////////
  // Ofsets for Camera mounts with Yaw only ////////////////////////////////////////////////////
  // Ofsets for Camera mounts with Yaw only ////////////////////////////////////////////////////

  private final double cameraOffsetX = 12.9; // CAD 13.0
  private final double cameraOffsetY =
      10.45; // by measurement to camera focal plane - ruler measurement was 9.9425
  private final double cameraOffsetZ =
      12.160; // CAD Measurement - with Z offset from measurements on  dashboard

  private final double rtCameraOffsetRoll = 0.0; // was.65
  private final double rtCameraOffsetPitch = -0.10;
  private final double rtCameraOffesetYaw = +23.285; // was 8.5 had about a 1.2 d error

  private final double ltCameraOffsetRoll = -0.40; // was 20
  private final double ltCameraOffsetPitch = -.25; // was +.2
  private final double ltCameraOffesetYaw = -22.665; // was 8.5

  public final Transform3d robotToRightFrtCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(cameraOffsetX), // verified correct, 12.8343
              Units.inchesToMeters(-cameraOffsetY), // verified negative is correct
              Units.inchesToMeters(cameraOffsetZ)), // verified correct
          new Rotation3d(
              Units.degreesToRadians(rtCameraOffsetRoll), // was -
              Units.degreesToRadians(
                  rtCameraOffsetPitch), // verified negative is correct -20.1 // 3/1 update: HAVE
              // NOT VERIFIED
              // (but robot drives fine) -
              // SZ
              Units.degreesToRadians(
                  rtCameraOffesetYaw))); // verified negative is correct - was 10.5

  public final Transform3d robotToLeftFrtCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(cameraOffsetX), // verified correct
              Units.inchesToMeters(cameraOffsetY), // verified correct
              Units.inchesToMeters(cameraOffsetZ)), // verified correct
          new Rotation3d(
              Units.degreesToRadians(ltCameraOffsetRoll), // was -
              Units.degreesToRadians(ltCameraOffsetPitch), // verified negative is correct
              Units.degreesToRadians(ltCameraOffesetYaw))); // verified positive is correct - was

  public final Transform3d rightcamtorobot =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-cameraOffsetX),
              Units.inchesToMeters(+cameraOffsetY),
              Units.inchesToMeters(-cameraOffsetZ)), // was -
          new Rotation3d(
              Units.degreesToRadians(-rtCameraOffsetRoll),
              Units.degreesToRadians(-rtCameraOffsetPitch),
              Units.degreesToRadians(-rtCameraOffesetYaw)));
  public final Transform3d leftcamtotobot =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-cameraOffsetX),
              Units.inchesToMeters(-cameraOffsetY),
              Units.inchesToMeters(-cameraOffsetZ)), // was -
          new Rotation3d(
              Units.degreesToRadians(-ltCameraOffsetRoll),
              Units.degreesToRadians(-ltCameraOffsetPitch),
              Units.degreesToRadians(-ltCameraOffesetYaw)));

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

  private final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  // TODO: Ensure that your camera is calibrated and 3D mode is enabled. Read
  // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/apriltag-pipelines/multitag.html#multitag-localization
  // TODO: // The field from AprilTagFields will be different depending on the game.
  //  private final AprilTagFieldLayout aprilTagFieldLayout =
  //      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  private final PhotonCamera rightFrtCamera;
  private final PhotonCamera leftFrtCamera;
  private final PhotonPoseEstimator rightFrtCamPoseEstimator;
  private final PhotonPoseEstimator leftFrtCamPoseEstimator;

  private Matrix<N3, N1> currightFrtCamStdDevs;
  private Matrix<N3, N1> curleftFrtCamStdDevs;

  private List<AprilTag> modifiedAprilTags;

  public VisionSubsystem() {
    // Creates new apriltag layout
    List<AprilTag> modifiedAprilTags = new ArrayList<>();

    AprilTag aprilTag6 =
        new AprilTag(
            6,
            new Pose3d(
                13.474446,
                3.3063179999999996,
                0.308102,
                new Rotation3d(
                    new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))));

    AprilTag aprilTag7 =
        new AprilTag(
            7,
            new Pose3d(
                13.890498, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

    AprilTag aprilTag8 =
        new AprilTag(
            8,
            new Pose3d(
                13.474446,
                4.745482,
                0.308102,
                new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994))));

    AprilTag aprilTag9 =
        new AprilTag(
            9,
            new Pose3d(
                12.643358,
                4.745482,
                0.308102,
                new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386))));

    AprilTag aprilTag10 =
        new AprilTag(
            10,
            new Pose3d(
                12.227305999999999,
                4.0259,
                0.308102,
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))));

    AprilTag aprilTag11 =
        new AprilTag(
            11,
            new Pose3d(
                12.643358,
                3.3063179999999996,
                0.308102,
                new Rotation3d(
                    new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387))));

    AprilTag aprilTag17 =
        new AprilTag(
            17,
            new Pose3d(
                4.073905999999999,
                3.3063179999999996,
                0.308102,
                new Rotation3d(
                    new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387))));

    AprilTag aprilTag18 =
        new AprilTag(
            18,
            new Pose3d(
                3.6576,
                4.0259,
                0.308102,
                new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))));

    AprilTag aprilTag19 =
        new AprilTag(
            19,
            new Pose3d(
                4.073905999999999,
                4.745482,
                0.308102,
                new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386))));

    AprilTag aprilTag20 =
        new AprilTag(
            20,
            new Pose3d(
                4.904739999999999,
                4.745482,
                0.308102,
                new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994))));

    AprilTag aprilTag21 =
        new AprilTag(
            21,
            new Pose3d(
                5.321046, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

    AprilTag aprilTag22 =
        new AprilTag(
            22,
            new Pose3d(
                4.904739999999999,
                3.3063179999999996,
                0.308102,
                new Rotation3d(
                    new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))));

    modifiedAprilTags.add(aprilTag6);
    modifiedAprilTags.add(aprilTag7);
    modifiedAprilTags.add(aprilTag8);
    modifiedAprilTags.add(aprilTag9);
    modifiedAprilTags.add(aprilTag10);
    modifiedAprilTags.add(aprilTag11);

    modifiedAprilTags.add(aprilTag17);
    modifiedAprilTags.add(aprilTag18);
    modifiedAprilTags.add(aprilTag19);
    modifiedAprilTags.add(aprilTag20);
    modifiedAprilTags.add(aprilTag21);
    modifiedAprilTags.add(aprilTag22);
    AprilTagFieldLayout newLayout = new AprilTagFieldLayout(modifiedAprilTags, 17.548, 8.052);

    // Create PhotonCamera objects
    rightFrtCamera = new PhotonCamera("rightFrtCamera");
    leftFrtCamera = new PhotonCamera("leftFrtCamera");
    rightFrtCamPoseEstimator =
        new PhotonPoseEstimator(newLayout, poseStrategy, robotToRightFrtCamera);
    leftFrtCamPoseEstimator =
        new PhotonPoseEstimator(newLayout, poseStrategy, robotToLeftFrtCamera);

    rightFrtCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    leftFrtCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    //
    // rightFrtCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    //
    // leftFrtCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

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

  double LeftCamNumOfTgts = 0;
  double RightCamNumOfTgts = 0;

  double TgtID = 99.0;
  // some variables to override photonVision "Best" with our own "best" (based on area only)
  double ltCamTgt0Area = 0.0;
  double ltCamTgt1Area = 0.0;
  double rtCamTgt0Area = 0.0;
  double rtCamTgt1Area = 0.0;

  // TA TODO: RIght now have only 1 Single Tag StdDev, s/b OK, need to calibrate multitag StdDevs!!!
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // These 2 constantly bounced off each other, try sligjtly lower values to average them
  // public static final Matrix<N3, N1> kMultiTagleftFrtCamStdDevs = VecBuilder.fill(0.025, 0.025,
  // .1);
  // public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs = VecBuilder.fill(0.025, 0.025,
  // .1);
  // <1, 1, 2> too slow - was .5,.5,1.0  - still a little slow and yaw mostly robot gyro

  // we used the below for most of Hoftstra - the more aggressive numbers below worked worse
  public static final Matrix<N3, N1> kMultiTagleftFrtCamStdDevs = VecBuilder.fill(0.1, 0.1, .25);
  public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs = VecBuilder.fill(0.1, 0.1, .25);

  // Still seems slow - doubling rate for translation - faster rate is more incorrect - autonomous
  // works worse.
  // public static final Matrix<N3, N1> kMultiTagleftFrtCamStdDevs = VecBuilder.fill(0.05, 0.05,
  // .125);
  // public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs = VecBuilder.fill(0.05, 0.05,
  // .125);
  // public static final Matrix<N3, N1> kMultiTagleftFrtCamStdDevs = VecBuilder.fill(0.025, 0.025,
  // .1);
  // public static final Matrix<N3, N1> kMultiTagrightFrtCamStdDevs =
  //     VecBuilder.fill(0.025, 0.025, .1);

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

    //    determineAprilTagToDriveTo();

    // Publish to SmartDashboard

    // SmartDashboard.putNumber("DriveTo AprilTag", TgtID);

    // SmartDashboard.putBoolean("RightFrtCam Targets?   ", rightFrtCameraHasTargets);
    // SmartDashboard.putBoolean("Mult RightFrtCam Trgts?", multiplerightFrtCameraTargets);
    //   SmartDashboard.putString("Mult rightFrtCam IDs", rightFrtCameraAllIdsString);
    // SmartDashboard.putNumber("BestRightFrtCam Tgt ID ", rightFrtCameraBestTargetId);
    // SmartDashboard.putNumber("BestRightFrtCam Tgt Area", rightFrtCameraBestTargetArea);

    // SmartDashboard.putBoolean("leftFrtCam Targets?   ", leftFrtCameraHasTargets);
    // SmartDashboard.putBoolean("Mult leftFrtCam Trgts?", multipleleftFrtCameraTargets);
    //   SmartDashboard.putString("Multiple leftFrtCam IDs", leftFrtCameaAllIdsString);
    // SmartDashboard.putNumber("BestleftFrtCam Tgt ID ", leftFrtCameraBestTargetId);
    // SmartDashboard.putNumber("BestleftFrtCam Tgt Area", leftFrtCameraBestTargetArea);
  }

  //////////// rightFrt Camera Getter methods to access target data
  public boolean rightFrtCamTgtDectected() {
    return rightFrtCameraHasTargets;
  }

  public double getRightFrtCamNumOfTgts() {
    return RightCamNumOfTgts;
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

  public double getLeftFrtCamNumOfTgts() {
    return LeftCamNumOfTgts;
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
      updateEstimationRightFrtCamStdDevs(visionEst, change.getTargets());
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
  private void updateEstimationRightFrtCamStdDevs(
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

      RightCamNumOfTgts = numTags;

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
      updateEstimationLeftFrtCamStdDevs(visionEst, change.getTargets());
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
  private void updateEstimationLeftFrtCamStdDevs(
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

      LeftCamNumOfTgts = numTags;

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

  // private void installNewLayout() {
  //   // try {
  //   //   Path path = FileSystems.getDefault().getPath("modifiedAprilTagLayout.json");
  //   //   newLayout = new AprilTagFieldLayout(path);
  //   //   newLayout = new AprilTagFieldLayout(<public AprilTag(3,pOS)>,)
  //   // } catch (IOException e) {
  //   //   e.printStackTrace();
  //   // }
  // }

  // public void determineAprilTagToDriveTo() {

  //   TgtID = 99.0;
  //   double leftCamBestId = 99.0;
  //   double rightCamBestId = 99.0;
  //   double leftCamBestIdArea = 0.0;
  //   double rightCamBestIdArea = 0.0;

  //   if (rightFrtCamTgtDectected()
  //       && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
  //       (DriverStation.getAlliance().isPresent()
  //               && (DriverStation.getAlliance().get() == Alliance.Red)
  //               && (getrightFrtCamBestTgtId() >= 6)
  //               && (getrightFrtCamBestTgtId() <= 11))
  //           || (DriverStation.getAlliance().isPresent()
  //               && (DriverStation.getAlliance().get() == Alliance.Blue)
  //               && (getrightFrtCamBestTgtId() >= 17)
  //               && (getrightFrtCamBestTgtId() <= 22)))) {

  //     rightCamBestId = getrightFrtCamBestTgtId();
  //     rightCamBestIdArea = getrightFrtCamBestTgtArea();
  //   }

  //   if (leftFrtCamTgtDectected()
  //       && ( // red reef april tags are numbers 6 - 11, blue are 17 - 22
  //       (DriverStation.getAlliance().isPresent()
  //               && (DriverStation.getAlliance().get() == Alliance.Red)
  //               && (getleftFrtCamBestTgtId() >= 6)
  //               && (getleftFrtCamBestTgtId() <= 11))
  //           || (DriverStation.getAlliance().isPresent()
  //               && (DriverStation.getAlliance().get() == Alliance.Blue)
  //               && (getleftFrtCamBestTgtId() >= 17)
  //               && (getleftFrtCamBestTgtId() <= 22)))) {

  //     leftCamBestId = getleftFrtCamBestTgtId();
  //     leftCamBestIdArea = getleftFrtCamBestTgtArea();
  //   }

  //   if (leftCamBestIdArea >= rightCamBestIdArea) TgtID = leftCamBestId;
  //   else TgtID = rightCamBestId;
  // }

  // public double getTrgtIdToDriveTo() {
  //   return TgtID;
  // }
}
