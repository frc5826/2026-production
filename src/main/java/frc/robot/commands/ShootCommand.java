package frc.robot.commands;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends LoggedCommand {
    private final ShootSubsystem shootSubsystem;
    private final CameraSubsystem cameraSubsystem;

    public ShootCommand(ShootSubsystem shootSubsystem, CameraSubsystem cameraSubsystem) {
        this.shootSubsystem = shootSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        super.addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double hubDistance = this.cameraSubsystem.getHubDistance();
        this.shootSubsystem.setGoalDistance(hubDistance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.shootSubsystem.stopShoot();
    }
}
