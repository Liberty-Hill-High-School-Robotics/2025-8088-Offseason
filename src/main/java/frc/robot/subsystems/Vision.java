package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
// READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

public class Vision extends SubsystemBase {

  // make sure the name in quotes is EXACTLY the same as it is in PV
  PhotonCamera Limelight = new PhotonCamera("Camera Module v1");

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
        if (bestTarget.getFiducialId() == 22) {
          SmartDashboard.putNumber("cameraX", bestTarget.getBestCameraToTarget().getX());
          SmartDashboard.putNumber("cameraY", bestTarget.getBestCameraToTarget().getY());
          SmartDashboard.putNumber("cameraZ", bestTarget.getBestCameraToTarget().getZ());
        }
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
