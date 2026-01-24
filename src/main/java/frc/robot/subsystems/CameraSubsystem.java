package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends SubsystemBase {
    private List <Camera> cameras = new ArrayList<>(5);
    private Field2d field = new Field2d();
    public CameraSubsystem () {
        SmartDashboard.putData("field",field);
        cameras.add(new Camera("Arducam_1",new Translation3d(0,0,0),new Rotation3d()));
    }
    private void getCameraValues(Camera camera) {
        List<PhotonPipelineResult> results = camera.photon.getAllUnreadResults();
        if (!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size()-1);
            if(result.getMultiTagResult().isPresent()){
                Transform3d cameraTransform = result.getMultiTagResult().get().estimatedPose.best;
                field.setRobotPose(Pose3d.kZero.plus(cameraTransform).toPose2d());
            }

        }

    }

    @Override
    public void periodic() {
        for(Camera c:cameras){
            getCameraValues(c);
        }
    }
    private record Camera(PhotonCamera photon,Transform3d robotToCamera){
        private Camera(String name, Translation3d translation, Rotation3d rotation){
            this(new PhotonCamera(name), new Transform3d(translation,rotation));
        }
    }
}


