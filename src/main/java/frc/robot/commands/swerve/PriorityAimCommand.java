package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.localization.Locations;
import frc.robot.subsystems.SwerveSubsystem;

public class PriorityAimCommand extends LoggedCommand {

    private SwerveSubsystem swerveSubsystem;
    private Rotation2d rotation2d = Rotation2d.kZero;

    public PriorityAimCommand(SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

    }

    @Override
    public void initialize() {
        super.initialize();
        Pose2d robotPose = swerveSubsystem.getPose();

        if (Locations.getAllianceZone().contains(robotPose.getTranslation())) {
            rotation2d = Locations.getHubPose().getTranslation().minus(robotPose.getTranslation()).getAngle();
            swerveSubsystem.setTurnGoal(rotation2d);
        } else if (Locations.getLeftSideMidZone().contains(robotPose.getTranslation())) {
            rotation2d = Locations.getLeftSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
            swerveSubsystem.setTurnGoal(rotation2d);
        } else if (Locations.getRightSideMidZone().contains(robotPose.getTranslation())) {
            rotation2d = Locations.getRightSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
            swerveSubsystem.setTurnGoal(rotation2d);
        }
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d robotPose = swerveSubsystem.getPose();
        Rotation2d rotation2dNew;

        if (Locations.getAllianceZone().contains(robotPose.getTranslation())) {
            rotation2dNew = Locations.getHubPose().getTranslation().minus(robotPose.getTranslation()).getAngle();
        } else if (Locations.getLeftSideMidZone().contains(robotPose.getTranslation())) {
            rotation2dNew = Locations.getLeftSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
        } else if (Locations.getRightSideMidZone().contains(robotPose.getTranslation())) {
            rotation2dNew = Locations.getRightSideTarget().getTranslation().minus(robotPose.getTranslation()).getAngle();
        } else
            return;

            rotation2d = rotation2dNew;
            swerveSubsystem.setTurnGoal(rotation2d);

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
