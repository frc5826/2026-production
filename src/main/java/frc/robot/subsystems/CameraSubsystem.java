package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> odometry;

    public CameraSubsystem(TriConsumer<Pose2d, Double, Matrix<N3, N1>> odometry) {

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
                    Pose3d robotPose = cameraPose.plus(camera.robotToCamera.inverse());
                    //TODO look at which ids were used and find avg distance to set stdDev
                    odometry.accept(robotPose.toPose2d(), result.getTimestampSeconds(), VecBuilder.fill(0.1,0.1,1));
                } else {
                    return;
                }
//                else {
//                    PhotonTrackedTarget target = result.getBestTarget();
//                    if (target.getPoseAmbiguity() > 0.2) {
//                        return;
//                    }
//                    cameraPose = layout.getTagPose(target.fiducialId).get().plus(target.bestCameraToTarget);
//
//                }
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

    @FunctionalInterface
    public interface TriConsumer<T, U, V> {
        void accept(T t, U u, V v);
    }
}


