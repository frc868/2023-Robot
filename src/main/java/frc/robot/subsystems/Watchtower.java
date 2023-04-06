package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AdvantageScopeSerializer;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.DoubleArrayLogItem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
                                    () -> AdvantageScopeSerializer
                                            .serializePose3ds(cam.getCurrentlyDetectedAprilTags()),
                                    LogLevel.MAIN),
                            new DoubleArrayLogItem("Detected Robot Pose3d",
                                    () -> AdvantageScopeSerializer
                                            .serializePose3ds(List.of(cam.getCurrentlyDetectedRobotPose())),
                                    LogLevel.MAIN)));
        }

        LoggingManager.getInstance()
                .addGroup(new LogGroup("Watchtower",
                        new DoubleArrayLogItem("Estimated Robot Pose",
                                () -> AdvantageScopeSerializer
                                        .serializePose2ds(List.of(this.poseEstimator.getEstimatedPosition())),
                                LogLevel.MAIN),
                        new DoubleArrayLogItem("Detected AprilTags",
                                this::getAllDetectedAprilTags, LogLevel.MAIN),
                        new DoubleArrayLogItem("Detected Robot Poses",
                                this::getAllDetectedRobotPoses, LogLevel.MAIN),
                        new DoubleArrayLogItem("Camera Poses", this::getAllCameraPoses,
                                LogLevel.MAIN)));

        // log the packed apriltag poses from each cam individually
        // log the robot's pose3d from each cam
    }

    public double[] getAllDetectedAprilTags() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTagPhotonCamera cam : photonCameras) {
            poses.addAll(cam.getCurrentlyDetectedAprilTags());
        }
        return AdvantageScopeSerializer.serializePose3ds(poses);
    }

    public double[] getAllDetectedRobotPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTagPhotonCamera cam : photonCameras) {
            if (cam.hasPose())
                poses.add(cam.getCurrentlyDetectedRobotPose());
        }
        return AdvantageScopeSerializer.serializePose3ds(poses);
    }

    public double[] getAllCameraPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (Transform3d transform : Constants.Vision.ROBOT_TO_CAMS) {
            poses.add(new Pose3d(poseEstimator.getEstimatedPosition()).plus(transform));
        }
        return AdvantageScopeSerializer.serializePose3ds(poses);
    }

    public void updatePoseEstimator() {
        if (poseEstimator != null && DriverStation.isTeleop()) {
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

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }
}
