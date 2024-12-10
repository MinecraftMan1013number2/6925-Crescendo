package frc.robot.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PhotonVisionHandler extends SubsystemBase {
    /**
     * Constants
    //  */
    private final double kCameraHeight = Units.inchesToMeters(27.0);
    private final double kCameraPitch = 0.0;
    private final double kTargetHeight = Units.inchesToMeters(1.0);

    private final RobotContainer instance;
    private final PhotonCamera photonCamera;

    public PhotonVisionHandler(RobotContainer instance, String cameraName) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.instance = instance;
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
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }

    public void outputData() {
        /*
         * double getYaw(): The yaw of the target in degrees (positive right).
         * double getPitch(): The pitch of the target in degrees (positive up).
         * double getArea(): The area (how much of the camera feed the bounding box takes up) as a percent (0-100).
         * double getSkew(): The skew of the target in degrees (counter-clockwise positive).
         * double[] getCorners(): The 4 corners of the minimum bounding box rectangle.
         * Transform2d getCameraToTarget(): The camera to target transform
         */
        PhotonTrackedTarget target = getClosestTarget();
        if (target != null) {
            SmartDashboard.putNumber("PV Yaw", round(target.getYaw()));
            SmartDashboard.putNumber("PV Pitch", round(target.getPitch()));
            SmartDashboard.putNumber("PV Area", round(target.getArea()));
            SmartDashboard.putNumber("PV Skew", round(target.getSkew()));

            SmartDashboard.putNumber("Camera dist (in)", Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight, kTargetHeight, kCameraPitch, Math.toRadians(target.getPitch()))));

            /**
             * DATA INFO
             * notes:
             *   - on cart (yaw, pitch) & on ground (yaw, pitch)
             *   - note in bottom left: (9.44, -31.51) & (5.5, 29.97)
             *   - note in top right: (-16.94, -14.83) & (-6.19, -12.37)
             *   - (~0, ~0): ?
             * yaw: left (more negative) / right (more positive)
             * pitch: forward (more negative) / backward (more positive)
             * area: closeness (bigger = closer, smaller = farther)
             * skew: ? (always 0)
             */
        }
    }

    private double round(double input) {
        return round(input, 2);
    }
    private double round(double input, int places) {
        double amt = Math.pow(10, places);
        return Math.round(input*amt)/amt;
    }

    @Override
    public void periodic() {
        outputData();
    }
}
