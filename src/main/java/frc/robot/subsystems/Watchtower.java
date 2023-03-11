package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.DoubleArrayLogItem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Watchtower extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator = null;

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

        for (AprilTagPhotonCamera cam : photonCameras) {
            LoggingManager.getInstance().addGroup(
                    new LogGroup("Watchtower/" + cam.getName(),
                            new DoubleArrayLogItem("Detected AprilTags",
                                    () -> packPosesToAdvantageScope(cam.getCurrentlyDetectedAprilTags()),
                                    LogLevel.MAIN),
                            new DoubleArrayLogItem("Detected Robot Pose3d",
                                    () -> packPosesToAdvantageScope(List.of(cam.getCurrentlyDetectedRobotPose())),
                                    LogLevel.MAIN)));
        }

        // log the packed apriltag poses from each cam individually
        // log the robot's pose3d from each cam
    }

    public void updatePoseEstimator() {
        if (DriverStation.isTeleopEnabled()) {
            if (poseEstimator != null) {
                Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();
                for (AprilTagPhotonCamera photonCamera : photonCameras) {
                    Optional<EstimatedRobotPose> result = photonCamera
                            .getEstimatedGlobalPose(prevEstimatedRobotPose);

                    if (result.isPresent()) {
                        EstimatedRobotPose camPose = result.get();

                        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                                camPose.timestampSeconds);
                    }
                }
            }

        }
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    private double[] packPosesToAdvantageScope(List<Pose3d> poses) {
        double[] data = new double[poses.size() * 7];
        for (int i = 0; i < poses.size(); i++) {
            data[i * 7] = poses.get(i).getX();
            data[i * 7 + 1] = poses.get(i).getY();
            data[i * 7 + 2] = poses.get(i).getZ();
            data[i * 7 + 3] = poses.get(i).getRotation().getQuaternion().getW();
            data[i * 7 + 4] = poses.get(i).getRotation().getQuaternion().getX();
            data[i * 7 + 5] = poses.get(i).getRotation().getQuaternion().getY();
            data[i * 7 + 6] = poses.get(i).getRotation().getQuaternion().getZ();
        }
        return data;
    }
}
