package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.AdvantageScopeSerializer;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@LoggedObject
public class Watchtower extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator = null;

    /** The PhotonVision cameras, used to detect the AprilTags. */

    @Log(name = "OV9281-01", groups = "Cameras")
    AprilTagPhotonCamera photonCamera1 = new AprilTagPhotonCamera("OV9281-01",
            Constants.Vision.ROBOT_TO_CAMS[0]);

    @Log(name = "OV9281-02", groups = "Cameras")
    AprilTagPhotonCamera photonCamera2 = new AprilTagPhotonCamera("OV9281-02",
            Constants.Vision.ROBOT_TO_CAMS[1]);

    @Log(name = "OV9281-03", groups = "Cameras")
    AprilTagPhotonCamera photonCamera3 = new AprilTagPhotonCamera("OV9281-03",
            Constants.Vision.ROBOT_TO_CAMS[2]);

    @Log(name = "OV9281-04", groups = "Cameras")
    AprilTagPhotonCamera photonCamera4 = new AprilTagPhotonCamera("OV9281-04",
            Constants.Vision.ROBOT_TO_CAMS[3]);

    private AprilTagPhotonCamera[] photonCameras = new AprilTagPhotonCamera[] {
            photonCamera1, photonCamera2, photonCamera3, photonCamera4 };

    @Log(name = "Overall Estimated Robot Pose")
    private Supplier<double[]> estimatedPoseSupp = () -> AdvantageScopeSerializer
            .serializePose2ds(List.of(this.poseEstimator.getEstimatedPosition()));

    public Watchtower() {
        AutoManager.getInstance().setPoseEstimatorCallback(this::updatePoseEstimator);
    }

    @Log(name = "Detected AprilTags")
    public double[] getAllDetectedAprilTags() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTagPhotonCamera cam : photonCameras) {
            poses.addAll(cam.getCurrentlyDetectedAprilTags());
        }
        return AdvantageScopeSerializer.serializePose3ds(poses);
    }

    @Log(name = "Detected Robot Poses")
    public double[] getAllDetectedRobotPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTagPhotonCamera cam : photonCameras) {
            if (cam.hasPose())
                poses.add(cam.getCurrentlyDetectedRobotPose());
        }
        return AdvantageScopeSerializer.serializePose3ds(poses);
    }

    @Log(name = "Camera Poses")
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
