package frc.robot.photonvision;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PhotonVisionHandler extends SubsystemBase {
    /**
     * Constants
     */
    private static final double kCameraHeight = 0.0;
    private static final double kCameraPitch = 0.0;
    private static final double kTargetHeight = 0.0;
    private static final double kTargetPitch = 0.0;

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
        String[] cornersAsStrings = new String[target.getDetectedCorners().size()];
        for (int i = 0; i < target.getDetectedCorners().size(); i++) {
            cornersAsStrings[i] = target.getDetectedCorners().get(i).toString();
        }
        SmartDashboard.putNumber("PV Yaw", target.getYaw());
        SmartDashboard.putNumber("PV Pitch", target.getPitch());
        SmartDashboard.putNumber("PV Area", target.getArea());
        SmartDashboard.putNumber("PV Skew", target.getSkew());
        SmartDashboard.putStringArray("PV Corners", cornersAsStrings);
        SmartDashboard.putString("PV CameraToTarget", target.getBestCameraToTarget().toString());
    }

    private boolean startedLogging = false;
    @Override
    public void periodic() {
        if (
            instance.s_Swerve.getPose().getTranslation().getX() == 0 ||
            instance.s_Swerve.getPose().getTranslation().getY() == 0 ||
            getClosestTarget() == null
        ) return;
        
        if (!startedLogging) {
            startedLogging = true;
            DriverStation.reportWarning("Started logging PV values to SmartDashboard!", false);
        }

        outputData();
    }

    // public void estimate() {
    //     PhotonTrackedTarget target = getClosestTarget();
    //     // Calculate robot's field relative pose
    //     Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
    //         kCameraHeight,
    //         kTargetHeight,
    //         kCameraPitch,
    //         kTargetPitch,
    //         Rotation2d.fromDegrees(-target.getYaw()),
    //         instance.s_Swerve.geRotation2d(),
    //         ,
    //         cameraToRobot
    //     );
    // }
}
