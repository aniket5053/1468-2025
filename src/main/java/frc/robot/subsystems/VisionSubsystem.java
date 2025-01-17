package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  // Create PhotonCamera objects
  private final PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCamera");
  private final PhotonCamera noteCamera = new PhotonCamera("NoteCamera");

  // Construct PhotonPoseEstimator
  private final Transform3d robotToAprilTagCamera =
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
  private final PoseStrategy poseStrategy =
      PoseStrategy
          .MULTI_TAG_PNP_ON_COPROCESSOR; // TODO: Ensure that your camera is calibrated and 3D mode
  // is enabled. Read
  // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/apriltag-pipelines/multitag.html#multitag-localization
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo
          .loadAprilTagLayoutField(); // TODO: // The field from AprilTagFields will be different
  // depending on the game.

  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, robotToAprilTagCamera);

  // Initialize variables (default if aprilTagResults is empty)
  private boolean aprilTagHasTargets = false;
  private boolean multipleAprilTags = false;
  private String aprilTagsAllIdsString = "Nope";
  private int aprilTagBestTargetId =
      9999; // This indicates that something is wrong with the AprilTag camera
  private double aprilTagTargetYaw = 0;
  private double aprilTagTargetPitch = 0;
  private double aprilTagTargetArea = 0;

  private boolean noteHasTargets = false;
  private boolean multipleNotes = false;
  private double noteTargetYaw = 0;
  private double noteTargetPitch = 0;
  private double noteTargetArea = 0;

  private boolean aprilTag7Detected = false;
  private double aprilTag7Yaw = 0;

  public VisionSubsystem() {
    // Additional initialization if needed
  }

  @Override
  public void periodic() {
    // AprilTag camera

    // Read in relevant data from the Camera
    var aprilTagResults = aprilTagCamera.getAllUnreadResults();
    if (!aprilTagResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var aprilTagResult = aprilTagResults.get(aprilTagResults.size() - 1);
      aprilTag7Detected = false; // reset to false after true

      if (aprilTagResult.hasTargets()) {
        // At least one AprilTag was seen by the camera
        aprilTagHasTargets = true;
        aprilTag7Detected = false; // reset to false after true

        for (var target : aprilTagResult.getTargets()) {
          if (target.getFiducialId() == 7) {
            aprilTag7Detected = true;
            aprilTag7Yaw = target.getYaw();
          }
        }

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> aprilTagTargets = aprilTagResult.getTargets();
        if (aprilTagTargets.size() > 1) {
          // Multiple AprilTags detected
          multipleAprilTags = true;
          aprilTagsAllIdsString = "Not finished yet oops";
        } else {
          // Only one AprilTag detected
          multipleAprilTags = false;
          aprilTagsAllIdsString = "Only one";
        }

        // Gets best target (for multiple targets)
        aprilTagBestTargetId = aprilTagResult.getBestTarget().getFiducialId();
        aprilTagTargetYaw = aprilTagResult.getBestTarget().getYaw();
        aprilTagTargetPitch = aprilTagResult.getBestTarget().getPitch();
        aprilTagTargetArea = aprilTagResult.getBestTarget().getArea();
      } else {
        // No AprilTags detected by camera
        aprilTagHasTargets = false;
        multipleAprilTags = false;
        aprilTagsAllIdsString = "Nope";
        aprilTagBestTargetId = 99;
        aprilTagTargetYaw = 0;
        aprilTagTargetPitch = 0;
        aprilTagTargetArea = 0;
      }
    }
    // Publish to SmartDashboard
    SmartDashboard.putBoolean("AprilTag Dectected?", aprilTagHasTargets);
    SmartDashboard.putBoolean("Multiple AprilTags?", multipleAprilTags);
    SmartDashboard.putString("Multiple AprilTag IDs", aprilTagsAllIdsString);
    SmartDashboard.putNumber("Best AprilTag ID", aprilTagBestTargetId);
    SmartDashboard.putNumber("AprilTag Yaw", aprilTagTargetYaw);
    SmartDashboard.putNumber("AprilTag Pitch", aprilTagTargetPitch);
    SmartDashboard.putNumber("AprilTag Area", aprilTagTargetArea);

    // Note Camera

    // Read in relevant data from the Camera
    var noteResults = noteCamera.getAllUnreadResults();
    if (!noteResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var noteResult = noteResults.get(noteResults.size() - 1);

      if (noteResult.hasTargets()) {
        // At least one note was seen by the camera
        noteHasTargets = true;

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> noteTargets = noteResult.getTargets();
        if (noteTargets.size() > 1) {
          // Multiple notes detected
          multipleNotes = true;
        } else {
          // Only one note detected
          multipleNotes = false;
        }

        // Gets best target (for multiple targets)
        noteTargetYaw = noteResult.getBestTarget().getYaw();
        noteTargetPitch = noteResult.getBestTarget().getPitch();
        noteTargetArea = noteResult.getBestTarget().getArea();
      } else {
        // No notes detected by camera
        noteHasTargets = false;
        multipleNotes = false;
        noteTargetYaw = 0;
        noteTargetPitch = 0;
        noteTargetArea = 0;
      }
    }
    // Publish to SmartDashboard
    SmartDashboard.putBoolean("Note Dectected?", noteHasTargets);
    SmartDashboard.putBoolean("Multiple Notes?", multipleNotes);
    SmartDashboard.putNumber("Note Yaw", noteTargetYaw);
    SmartDashboard.putNumber("Note Pitch", noteTargetPitch);
    SmartDashboard.putNumber("Note Area", noteTargetArea);
  }

  // Getter methods to access target data
  public boolean aprilTagDectected() {
    return aprilTagHasTargets;
  }

  public boolean specificAprilTagDetected(int aprilTagId) {
    if (aprilTagId == 7) {
      return aprilTag7Detected;
    }
    return false;
  }

  public double getSpecificAprilTagYaw(int aprilTagId) {
    return aprilTag7Yaw;
  }

  public boolean noteDetected() {
    return noteHasTargets;
  }

  public double getNoteTargetYaw() {
    return noteTargetYaw;
  }

  public double getNoteTargetArea() {
    return noteTargetArea;
  }
}
