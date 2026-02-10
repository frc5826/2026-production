package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.subsystems.*;

public class CommandGroups {
    private CameraSubsystem camera;
    private ClimbSubsystem climb;
    private HoodSubsystem hood;
    private ConveyorSubsystem conveyor;
    private IntakeSubsystem intake;
    private ShootSubsystem shoot;
    private SwerveSubsystem swerve;
    private IndexSubsystem index;


    private String[] autoNames = new String[]{
             "shootOnly"
    };
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    public CommandGroups(CameraSubsystem camera, ClimbSubsystem climb, HoodSubsystem hood, ConveyorSubsystem conveyor, IntakeSubsystem intake, ShootSubsystem shoot, SwerveSubsystem swerve, IndexSubsystem index) {
        this.camera = camera;
        this.climb = climb;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intake = intake;
        this.shoot = shoot;
        this.swerve = swerve;
        this.index = index;


        for (String name : autoNames) {
            autoChooser.addOption(name, name);
        }
        autoChooser.setDefaultOption("empty","empty");

        SmartDashboard.putData("5826/Auto",autoChooser);
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
        return getSpinUpAim().andThen(new PriorityAimCommand(swerve, camera),
                shoot.getShootCommand(camera::getHubDistance, true), getIndeyor())
                .finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getSpinUpAim() {
        return shoot.getShootCommand(camera::getHubDistance, true)
                .alongWith(
                        new PriorityAimCommand(swerve, camera)
                ).until(() -> swerve.isAtTurnTarget() && shoot.isAtGoalSpeed());
    }
    public Command getIndeyor(){
        return conveyor.getConveyorCommand().alongWith(index.getIndexCommand());
    }

}

















































