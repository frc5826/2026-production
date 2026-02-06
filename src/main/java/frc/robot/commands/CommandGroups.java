package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.subsystems.*;

public class CommandGroups {
    private CameraSubsystem camera;
    private ClimbSubsystem climb;
    private HoodSubsystem hood;
    private IndexSubsystem index;
    private IntakeSubsystem intake;
    private LoggedCommand logged;
    private ShootSubsystem shoot;
    private SwerveSubsystem swerve;

    private String[] autoNames = new String[]{
            "empty", "shootOnly"
    };
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    public CommandGroups() {
        for (String name : autoNames) {
            autoChooser.addOption(name, name);
        }
    }

    public Command getAuto() {
        //Things that happen every time in auto go in init.
        //todo
        Command init = new InstantCommand();
        if (autoChooser.getSelected().equals("empty")) {
            return init;
        } else if (autoChooser.getSelected().equals("shootOnly")) {
            return init.alongWith(getShootGroup());

        }
        return new PrintCommand("SomthingTerribleHappenedAndSomthingBroke❌💔");
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen( new PriorityAimCommand(swerve,camera),
                shoot.getShootCommand(camera::getHubDistance,true),index.getIndexCommand(0.3)
        ).finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getSpinUpAim (){
        return shoot.getShootCommand(camera::getHubDistance,true)
                .alongWith(
                        new PriorityAimCommand(swerve,camera)
                ).until(()-> swerve.isAtTurnTarget()&& shoot.isAtGoalSpeed());
    }

}

