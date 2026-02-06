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
        return new PrintCommand("Something Broke");
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen(
                new ShootCommand(shoot,camera),index.getIndexCommand(0.3)
        ).until(shoot::isDoneShooting);
    }

    public Command getSpinUpAim (){
        return new ShootCommand(shoot,camera).until(shoot::isAtGoalSpeed)
                .alongWith(
                        new PriorityAimCommand(swerve,camera).until(swerve::isAtTurnTarget)
                );
    }

}

