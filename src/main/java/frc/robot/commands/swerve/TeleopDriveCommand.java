package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDriveCommand extends LoggedCommand {
    private SwerveSubsystem swerve;
    private XboxController xboxController;
    public TeleopDriveCommand (SwerveSubsystem swerve, XboxController xboxController) {
        this.swerve = swerve;
        this.xboxController = xboxController;
        addRequirements(swerve);
        System.out.println(getRequirements());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double xSpeed = -xboxController.getLeftY();
        double ySpeed = -xboxController.getLeftX();
        double turnSpeed = -xboxController.getRightX();

        if (Math.abs(xSpeed)<0.06){
            xSpeed = 0;
        }
        if(Math.abs(ySpeed)<0.06){
            ySpeed = 0;
        }
        if(Math.abs(turnSpeed)<0.06){
            turnSpeed = 0;
        }

        xSpeed = Math.signum(xSpeed)*Math.pow(xSpeed,2);
        ySpeed = Math.signum(ySpeed)*Math.pow(ySpeed,2);
        turnSpeed = Math.signum(turnSpeed)*Math.pow(turnSpeed,2);


        double multiplier = swerve.getMaxSpeed();
        swerve.teleopDrive(xSpeed * multiplier, ySpeed * multiplier, turnSpeed * multiplier);

    }
}
































