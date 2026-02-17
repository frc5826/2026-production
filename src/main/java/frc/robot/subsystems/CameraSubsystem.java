package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.localization.Locations;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

public class CameraSubsystem extends SubsystemBase {
    private List<Camera> cameras = new ArrayList<>(5);
    private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private BiConsumer<Pose2d, Double> odometry;

    public CameraSubsystem(BiConsumer<Pose2d, Double> odometry) {

        this.odometry = odometry;
        cameras.add(
                new Camera("Arducam_1", new Translation3d(0.3225, 0.0025, 0.29), new Rotation3d(0, -Math.toRadians(9), 0))
        );
    }

    private void getCameraValues(Camera camera) {
        List<PhotonPipelineResult> results = camera.photon.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            Pose3d cameraPose;
            if (result.hasTargets()) {
                if (result.getMultiTagResult().isPresent()) {
                    cameraPose = Pose3d.kZero.plus(result.getMultiTagResult().get().estimatedPose.best);
                } else {
                    PhotonTrackedTarget target = result.getBestTarget();
                    if (target.getPoseAmbiguity() > 0.2) {
                        return;
                    }
                    cameraPose = layout.getTagPose(target.fiducialId).get().plus(target.bestCameraToTarget);

                }
                Pose3d robotPose = cameraPose.plus(camera.robotToCamera.inverse());
                SmartDashboard.putNumberArray("5826/swerve/cameraPose", new double[]{cameraPose.getX(), cameraPose.getY(), cameraPose.getZ()});
                SmartDashboard.putNumberArray("5826/swerve/robotPose", new double[]{robotPose.getX(), robotPose.getY(), robotPose.getZ()});
                odometry.accept(robotPose.toPose2d(), result.getTimestampSeconds());
            }
        }

    }


    @Override
    public void periodic() {
        for (Camera c : cameras) {
            getCameraValues(c);
        }
    }

    private record Camera(PhotonCamera photon, Transform3d robotToCamera) {
        private Camera(String name, Translation3d translation, Rotation3d rotation) {
            this(new PhotonCamera(name), new Transform3d(translation, rotation));
        }
    }

}


