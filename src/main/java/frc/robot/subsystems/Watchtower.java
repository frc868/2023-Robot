package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Watchtower extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator = null;

    /** The PhotonVision cameras, used to detect the AprilTags. */
    private AprilTagPhotonCamera[] photonCameras = new AprilTagPhotonCamera[] {
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[0],
                    Constants.Vision.ROBOT_TO_CAMS[0]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[1],
                    Constants.Vision.ROBOT_TO_CAMS[1]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[2],
                    Constants.Vision.ROBOT_TO_CAMS[2]),
            new AprilTagPhotonCamera(Constants.Vision.CAMERA_NAMES[3],
                    Constants.Vision.ROBOT_TO_CAMS[3])
    };

    public Watchtower() {
        AutoManager.getInstance().setPoseEstimatorCallback(this::updatePoseEstimator);
    }

    public void updatePoseEstimator() {
        if (Constants.IS_USING_CAMERAS) {
            if (poseEstimator != null) {
                Pose2d estimatedPosition = poseEstimator.getEstimatedPosition();
                Field2d field = AutoManager.getInstance().getField();
                for (int i = 0; i < photonCameras.length; i++) {
                    Optional<EstimatedRobotPose> result = photonCameras[i]
                            .getEstimatedGlobalPose(estimatedPosition);

                    FieldObject2d fieldObject = field.getObject("apriltag_cam" + i +
                            "_est_pose");
                    if (result.isPresent()) {
                        EstimatedRobotPose camPose = result.get();
                        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                                camPose.timestampSeconds);
                        fieldObject.setPose(camPose.estimatedPose.toPose2d());
                    } else {
                        // move it way off the screen to make it disappear
                        fieldObject.setPose(new Pose2d(-100, -100, new Rotation2d()));
                    }
                }
            }

        }
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

}
