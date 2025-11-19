package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
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
  PhotonCamera Limelight = new PhotonCamera("Camera_Module_v1");
  public final Pigeon2 gyro = new Pigeon2(CanIDs.GyroID);
  private final Field2d field = new Field2d();

  public Vision() {
    // initalization here
    // none needed ^
  }

  @Override
  public void periodic() {
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

        PhotonPoseEstimator photonEstimator =
            new PhotonPoseEstimator(
                Constants.AutoConstants.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.AutoConstants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        EstimatedRobotPose photonPoseTracked =
            photonEstimator.update(Limelight.getLatestResult()).get();
        Pose2d VisionPose =
            new Pose2d(
                photonPoseTracked.estimatedPose.getX(),
                photonPoseTracked.estimatedPose.getY(),
                gyro.getRotation2d());
        field.setRobotPose(VisionPose);
        SmartDashboard.putData("Field", field);
      }

    } else {
      SmartDashboard.putBoolean("TARGET", false);
    }
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
}
