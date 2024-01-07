package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Vision.*;

@LoggedObject
public class Watchtower extends SubsystemBase implements BaseVision {
    private SwerveDrivePoseEstimator poseEstimator = null;
    private Supplier<Pose2d> simPoseSupplier = null;
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera ov9281_01 = new AprilTagPhotonCamera("OV9281-01", ROBOT_TO_CAMS[0],
            CAMERA_CONSTANTS, 0.64, 0.22);

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera ov9281_02 = new AprilTagPhotonCamera("OV9281-02", ROBOT_TO_CAMS[1],
            CAMERA_CONSTANTS, 0.64, 0.22);

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera ov9281_03 = new AprilTagPhotonCamera("OV9281-03", ROBOT_TO_CAMS[2],
            CAMERA_CONSTANTS, 0.64, 0.22);

    private AprilTagPhotonCamera[] photonCameras = new AprilTagPhotonCamera[] {
            ov9281_01, ov9281_02, ov9281_03 };

    public Watchtower() {
        AprilTagFieldLayout tagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        visionSim.addAprilTags(tagLayout);
        for (AprilTagPhotonCamera photonCamera : photonCameras) {
            visionSim.addCamera(photonCamera.getSim(), photonCamera.getRobotToCam());
        }
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(simPoseSupplier.get());
    }

    @Override
    public void updatePoseEstimator() {
        if (poseEstimator == null) {
            return;
        }

        Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();
        for (AprilTagPhotonCamera photonCamera : photonCameras) {
            Optional<EstimatedRobotPose> result = photonCamera
                    .getEstimatedGlobalPose(prevEstimatedRobotPose);

            if (result.isPresent()) {
                EstimatedRobotPose estPose = result.get();
                Matrix<N3, N1> stddevs = photonCamera.getEstimationStdDevs(estPose.estimatedPose.toPose2d(),
                        SINGLE_TAG_STD_DEVS, MULTI_TAG_STD_DEVS);
                poseEstimator.addVisionMeasurement(estPose.estimatedPose.toPose2d(),
                        estPose.timestampSeconds, stddevs);
            }
        }
    }

    @Override
    @Log
    public Pose3d[] getCameraPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (Transform3d transform : Constants.Vision.ROBOT_TO_CAMS) {
            poses.add(new Pose3d(poseEstimator.getEstimatedPosition()).plus(transform)
                    .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI))));
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    @Override
    @Log
    public Pose3d[] getAprilTagPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTag tag : AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTags()) {
            poses.add(tag.pose);
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    @Override
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
    }
}
