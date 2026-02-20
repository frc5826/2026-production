package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.math.localization.Locations;
import frc.robot.subsystems.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class CommandGroups {
    private CameraSubsystem camera;
    private ClimbSubsystem climb;
    private HoodSubsystem hood;
    private ConveyorSubsystem conveyor;
    private IntakeSubsystem intake;
    private ShootSubsystem shoot;
    private SwerveSubsystem swerve;
    private IndexSubsystem index;
    private SensorSubsystem sensor;


    private String[] autoNames = new String[]{
            "shootOnly",
            "depotGrab+Shoot",
            "humanPlayerGrab+Shoot",
            "runOutToMiddle+Shoot"
    };
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    public CommandGroups(CameraSubsystem camera, ClimbSubsystem climb, HoodSubsystem hood, ConveyorSubsystem conveyor, IntakeSubsystem intake, ShootSubsystem shoot, SwerveSubsystem swerve, IndexSubsystem index, SensorSubsystem sensor) {
        this.camera = camera;
        this.climb = climb;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intake = intake;
        this.shoot = shoot;
        this.swerve = swerve;
        this.index = index;
        this.sensor = sensor;


        for (String name : autoNames) {
            autoChooser.addOption(name, name);
        }
        autoChooser.setDefaultOption("empty", "empty");

        SmartDashboard.putData("5826/Auto", autoChooser);


    }

    public Command getAuto() {
        //Things that happen every time in auto go in init.
        //todo
        Command init = new InstantCommand().alongWith(intake.intakeDown());




        return init;
//        if (autoChooser.getSelected().equals("empty")) {
//            return init;
//        } else if (autoChooser.getSelected().equals("shootOnly")) {
//            return init.alongWith(getShootGroup());
//
//        } else if (autoChooser.getSelected().equals("depotGrab+Shoot")) {
//            return init.alongWith(intake.getIntakeCommand(), getPathCommand("toDepotCommand"))
//                    .andThen(getPathCommand("awayDepotCommand"))
//                    .andThen(getShootGroup());
//
//        } else if (autoChooser.getSelected().equals("humanPlayerGrab+Shoot")) {
//            return AutoBuilder.buildAuto("toHumanPlayerPath").until(sensor.getBeamBreak())
//                    .andThen(AutoBuilder.buildAuto("awayHumanPlayerPath"))
//                    .andThen(getShootGroup());
//
//        } else if (autoChooser.getSelected().equals("runOutToMiddle+Shoot")) {
//            if (Locations.getLeftAllianceZonePose().contains(swerve.getPose().getTranslation()))
//                return getInteyor().alongWith(AutoBuilder.buildAuto("toMiddleFromLeftPath").andThen(getShootGroup()));
//            else if (Locations.getRightAllianceZonePose().contains(swerve.getPose().getTranslation())) {
//                return getInteyor().alongWith(AutoBuilder.buildAuto("toMiddleFromRightPath").andThen(getShootGroup()));
//            }
//
//        }

    }

    public Command getPathCommand(String commandName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(commandName));
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen(Commands.parallel(
                new PriorityAimCommand(swerve, camera),
                shoot.getShootCommand(swerve::getHubDistance, true),
                getInteyor()
        )).finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getDumbShootGroup() {
        return shoot.getShootCommand(3000).andThen(getInteyor())
                .finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getSpinUpAim() {
        return shoot.getShootCommand(swerve::getHubDistance, true)
                .alongWith(
                        new PriorityAimCommand(swerve, camera)
//                ).until(()->shoot.isAtGoalSpeed());
                ).until(() -> swerve.isAtTurnTarget() && shoot.isAtGoalSpeed());
    }

    public Command getInteyor() {
        return conveyor.getConveyorCommand().alongWith(index.getIndexCommand(), intake.getIntakeCommand());
    }
}

















































