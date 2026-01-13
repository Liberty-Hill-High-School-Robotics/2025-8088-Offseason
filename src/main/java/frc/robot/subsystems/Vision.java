package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
// READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

public class Vision extends SubsystemBase {

  // make sure the name in quotes is EXACTLY the same as it is in PV
  PhotonCamera AprilTagCam = new PhotonCamera("AprilTagCam");
  private final Field2d field = new Field2d();
  private final EstimateConsumer estConsumer;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    photonEstimator =
        new PhotonPoseEstimator(
            Constants.VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.kRobotToCam);
  }

  @Override
  public void periodic() {

    // Cheack every tag to estimate Pose
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (PhotonPipelineResult change : AprilTagCam.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            field.setRobotPose(est.estimatedPose.toPose2d());
            SmartDashboard.putData("Field", field);
          });

      if (change.hasTargets()) {
        SmartDashboard.putBoolean("TARGET", true);
        PhotonTrackedTarget bestTarget = change.getBestTarget();
        SmartDashboard.putNumber("cameraX", bestTarget.getBestCameraToTarget().getX());
        SmartDashboard.putNumber("cameraY", bestTarget.getBestCameraToTarget().getY());
        SmartDashboard.putNumber("cameraZ", bestTarget.getBestCameraToTarget().getZ());
      } else {
        SmartDashboard.putBoolean("TARGET", false);
      }
    }
    /* OLD
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches, etc
    // You can retrieve the latest pipeline result using the PhotonCamera instance."
    // Check if the latest result has any targets.
    // GetLL Values
    if (Limelight.isConnected()) {
      PhotonPipelineResult result = Limelight.getLatestResult();
      // Get the current best target.

      if (result.hasTargets()) {
        SmartDashboard.putBoolean("TARGET", true);
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        SmartDashboard.putNumber("cameraX", bestTarget.getBestCameraToTarget().getX());
        SmartDashboard.putNumber("cameraY", bestTarget.getBestCameraToTarget().getY());
        SmartDashboard.putNumber("cameraZ", bestTarget.getBestCameraToTarget().getZ());

        // Pose estimation
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        EstimatedRobotPose photonPoseTracked =
            photonEstimator.update(Limelight.getLatestResult()).get();
        Pose2d VisionPose = photonPoseTracked.estimatedPose.toPose2d();
        field.setRobotPose(VisionPose);
        SmartDashboard.putData("Field", field);

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(bestTarget.getBestCameraToTarget().getX(), 2.0) / 1; // Assumes only one tag
        double linearStdDev = Constants.VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = Constants.VisionConstants.angularStdDevBaseline * stdDevFactor;

        // Send pose to drive
        consumer.accept(
            VisionPose,
            photonPoseTracked.timestampSeconds,
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

    } else {
      SmartDashboard.putBoolean("TARGET", false);
    }
    */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  // as well as check for limits and reset encoders,
  // return true/false if limit is true, or encoder >= x value

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  private Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // Sends Pose to Drive
  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
