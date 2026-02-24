package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.math.localization.Locations;
import frc.robot.subsystems.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.nio.file.Path;

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

        if (autoChooser.getSelected().equals("empty")) {
            return init;
        } else if (autoChooser.getSelected().equals("shootOnly")) {
            return init.andThen(moveCommand(-2, 0)).andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("depotGrab+Shoot")) {
            return init.alongWith(intake.getIntakeCommand(), getPathCommand("toDepotCommand"))
                    .andThen(getPathCommand("awayDepotCommand"))
                    .andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("humanPlayerGrab+Shoot")) {
            return AutoBuilder.buildAuto("toHumanPlayerPath").until(sensor.getBeamBreak())
                    .andThen(AutoBuilder.buildAuto("awayHumanPlayerPath"))
                    .andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("runOutToMiddle+Shoot")) {
            if (Locations.getLeftAllianceZonePose().contains(swerve.getPose().getTranslation()))
                return getInteyor().alongWith(AutoBuilder.buildAuto("toMiddleFromLeftPath").andThen(getShootGroup()));
            else if (Locations.getRightAllianceZonePose().contains(swerve.getPose().getTranslation())) {
                return getInteyor().alongWith(AutoBuilder.buildAuto("toMiddleFromRightPath").andThen(getShootGroup()));
            }

        }
        return init;
    }

    public Command getPathCommand(String commandName) {
        try {
            return LoggedCommand.logCommand(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(commandName)),
                    "followPathCommand"
            );
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

    public Command gotoCommand(Pose2d endPose){
        Command c = new Command() {
            Command pathCommand;
            {addRequirements(swerve);}

            @Override
            public void initialize() {
                var startPose = swerve.getPose();
                var waypoints = PathPlannerPath.waypointsFromPoses(
                        new Pose2d(startPose.getX(), startPose.getY(), Locations.angleTo(startPose, endPose)),
                        new Pose2d(endPose.getX(), endPose.getY(), Locations.angleTo(startPose, endPose))
                );
                var path = new PathPlannerPath(
                        waypoints,
                        new PathConstraints(2, 4, 3, 6),
                        new IdealStartingState(0, startPose.getRotation()),
                        new GoalEndState(0, endPose.getRotation())
                );
                path.preventFlipping = true;
                pathCommand = AutoBuilder.followPath(path);
                pathCommand.initialize();
            }

            @Override
            public void execute() {
                pathCommand.execute();
            }

            @Override
            public void end(boolean interrupted) {
                pathCommand.end(interrupted);
            }

            @Override
            public boolean isFinished() {
                return pathCommand.isFinished();
            }
        };
        return LoggedCommand.logCommand(c, "Drive to " + endPose);
    }

    public Command moveCommand(double x, double y){
        Command c = new Command() {
            Command pathCommand;
            {addRequirements(swerve);}

            @Override
            public void initialize() {
                var startPose = swerve.getPose();
                var endPose = Locations.move(startPose, x, y);
                var waypoints = PathPlannerPath.waypointsFromPoses(
                        new Pose2d(startPose.getX(), startPose.getY(), Locations.angleTo(startPose, endPose)),
                        new Pose2d(endPose.getX(), endPose.getY(), Locations.angleTo(startPose, endPose))
                );
                var path = new PathPlannerPath(
                        waypoints,
                        new PathConstraints(2, 4, 3, 6),
                        new IdealStartingState(0, startPose.getRotation()),
                        new GoalEndState(0, endPose.getRotation())
                );
                path.preventFlipping = true;
                pathCommand = AutoBuilder.followPath(path);
                pathCommand.initialize();
            }

            @Override
            public void execute() {
                pathCommand.execute();
            }

            @Override
            public void end(boolean interrupted) {
                pathCommand.end(interrupted);
            }

            @Override
            public boolean isFinished() {
                return pathCommand.isFinished();
            }
        };
        return LoggedCommand.logCommand(c, "move ("+x+","+y+")");
    }
    public Command getDejammerCommand(){
       return intake.getReverseIntakeCommand().alongWith(conveyor.getReverseConveyorCommand()).withTimeout(1);
    }
}