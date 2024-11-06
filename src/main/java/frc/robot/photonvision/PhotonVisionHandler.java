package frc.robot.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PhotonVisionHandler extends PhotonCamera {
    /**
     * Constants
     */
    private static final double kCameraHeight = 0.0;
    private static final double kCameraPitch = 0.0;
    private static final double kTargetHeight = 0.0;
    private static final double kTargetPitch = 0.0;

    public PhotonVisionHandler(String cameraName) {
        super(cameraName);
    }
    /*
     * NOTES
     * camera.getLatestResult() --> PhotonPipelineResult
     * 
     * TODO
     * - static IP (https://docs.photonvision.org/en/latest/docs/installation/networking.html#digital-networking)
     * - The address in the code above (photonvision.local) is the hostname of the coprocessor.
     *     This can be different depending on your hardware, and can be checked in the settings tab under “hostname”.
     *     PortForwarder.add(5800, "photonvision.local", 5800);
     * 
     * - Ensure that exposure is as low as possible
     * - Ensure that you don’t have the dashboard up when you don’t need it to reduce bandwidth
     * - Stream at as low of a resolution as possible while still detecting AprilTags to stay within field bandwidth limits.
     */

    public PhotonTrackedTarget getClosestTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }

    public void estimate() {
        // Calculate robot's field relative pose
        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
  kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);
    }
}
