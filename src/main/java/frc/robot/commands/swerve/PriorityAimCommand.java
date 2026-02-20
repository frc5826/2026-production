package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.math.localization.Locations;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonUtils;

public class PriorityAimCommand extends LoggedCommand {

    private CameraSubsystem cameraSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public PriorityAimCommand(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem){

        this.swerveSubsystem = swerveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

    }

    @Override
    public void initialize() {
        super.initialize();
        Pose2d robotPose = swerveSubsystem.getPose();
        Rotation2d rotation2d = Locations.getHubPose().getTranslation().minus(robotPose.getTranslation()).getAngle();
        SmartDashboard.putNumber("angle",rotation2d.getDegrees());
        swerveSubsystem.setTurnGoal(rotation2d);


//        if (Locations.getAllianceZonePose().contains(robotPose.getTranslation())) {
//
//        }
//        else if (Locations.getLeftSideMidPose().contains(robotPose.getTranslation())) {
//            Rotation2d rotation2d = Locations.getLeftSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
//            swerveSubsystem.setTurnGoal(rotation2d);
//        }
//        else if (Locations.getRightSideMidPose().contains(robotPose.getTranslation())) {
//            Rotation2d rotation2d = Locations.getRightSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
//            swerveSubsystem.setTurnGoal(rotation2d);
//        }

    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putBoolean("5826/priorityaim/isFinished", isFinished());


    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        swerveSubsystem.endTurn();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
