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

import java.util.function.Supplier;

import static frc.robot.Constants.Swerve.*;

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
            "shootOnly",
            "depotGrab+Shoot",
            "humanPlayerGrab+Shoot",
            "runOutToMiddle+Shoot",
            "middleShootHumanShoot",
            "middleShootClimb"
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
            return init.andThen(moveCommand(-2, 0, cSlowPath, 0)).andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("depotGrab+Shoot")) {
            return init.alongWith(intake.getIntakeCommand(), getPathCommand("toDepotCommand"))
                    .andThen(getPathCommand("awayDepotCommand"))
                    .andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("humanPlayerGrab+Shoot")) {
            return getPathCommand("toHumanPlayerPath")
                    .andThen(new WaitUntilCommand(() -> shoot.getCANRange()))
                    .andThen(getPathCommand("awayHumanPlayerPath"))
                    .andThen(getShootGroup());

        } else if (autoChooser.getSelected().equals("runOutToMiddle+Shoot")) {
            return init.alongWith(getMidAuto());
        } else if (autoChooser.getSelected().equals("middleShootHumanShoot")) {
            return init.alongWith((getPathCommand("midFromRightAuto"))
                            .andThen(getPathCommand("rightFromMidAutoAlter")).alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getShootGroup().withTimeout(4))
                    .andThen(getPathCommand("humanFromRightAuto").deadlineFor(intake.getIntakeCommand()))
                    .andThen(getShootGroup().withTimeout(4));
        } else if (autoChooser.getSelected().equals("middleShootClimb")) {
            return init.alongWith((getPathCommand("midFromLeftAuto"))
                            .andThen(getPathCommand("leftFromMidAutoAlter")).alongWith(shoot.getShootCommand(3100)).deadlineFor(intake.getIntakeCommand()))
                    .andThen(getShootGroup().withTimeout(4))
                    .andThen(getPathCommand("ClimbLeft").alongWith(climb.hookUpCommand()))
                    .andThen(climb.hookDownCommand().repeatedly().finallyDo(() -> climb.hookUp()));
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

    public Command getMidAuto() {
        if (Locations.getLeftAutoZone().contains(swerve.getPose().getTranslation())) {
            return (//First Pass
                    getPathCommand("midFromLeftAuto"))
                        .andThen(getPathCommand("leftFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                        .andThen(getShootGroup().withTimeout(10))
                    //Second Pass
                    .andThen(getPathCommand("finalMidFromLeftAuto")
                            .andThen(getPathCommand("finalLeftFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                            .andThen(getShootGroup().withTimeout(10))
                    );
        } else if (Locations.getRightAutoZone().contains(swerve.getPose().getTranslation())) {
            return (//First Pass
                    getPathCommand("midFromRightAuto"))
                    .andThen(getPathCommand("rightFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getShootGroup().withTimeout(10))
                    //Second Pass
//                    .andThen(getPathCommand("finalMidFromRightAuto")
//                            .andThen(getPathCommand("finalRightFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
//                            .andThen(getShootGroup().withTimeout(10)))
                    ;
        }
        return new InstantCommand();
    }

    public Command getPathDriveTestCommand() {
        return moveCommand(1.5, 0, cSlowPath, 0).andThen(
                moveCommand(0, 1.5, cSlowPath, 0),
                moveCommand(-1.5, 0, cSlowPath, 0),
                moveCommand(0, -1.5, cSlowPath, 0)
        );
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen(Commands.parallel(
                new PriorityAimCommand(swerve),
                shoot.getShootCommand(swerve::getHubDistance, true),
                getInteyor()
        )).finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getDumbShootGroup() {
        return shoot.getShootCommand(3600)
                .andThen(getInteyor())
                .finallyDo(shoot::stopShoot);
        //.until(shoot::isDoneShooting)
    }

    public Command getSpinUpAim() {
        return shoot.getShootCommand(swerve::getHubDistance, true)
                .alongWith(
                        new PriorityAimCommand(swerve)
//                ).until(()->shoot.isAtGoalSpeed());
                ).until(() -> swerve.isAtTurnTarget() && shoot.isAtGoalSpeed());
    }

    public Command getInteyor() {
        return conveyor.getConveyorCommand().alongWith(index.getIndexCommand().alongWith(intake.getIntakeCommand()));
    }

    public Command getDejammerCommand() {
        return intake.getReverseIntakeCommand().alongWith(index.getReverseIndexCommand()).alongWith(conveyor.getReverseConveyorCommand()).withTimeout(1);
    }

    public Command gotoCommand(Supplier<Pose2d> endPose, PathConstraints constraints, double endSpeed) {
        Command c = new Command() {
            Command pathCommand;

            {
                addRequirements(swerve);
            }

            @Override
            public void initialize() {
                var startPose = swerve.getPose();
                var waypoints = PathPlannerPath.waypointsFromPoses(
                        new Pose2d(startPose.getX(), startPose.getY(), Locations.angleTo(startPose, endPose.get())),
                        new Pose2d(endPose.get().getX(), endPose.get().getY(), Locations.angleTo(startPose, endPose.get()))
                );
                var path = new PathPlannerPath(
                        waypoints,
                        constraints,
                        new IdealStartingState(0, startPose.getRotation()),
                        new GoalEndState(endSpeed, endPose.get().getRotation())
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

    public Command moveCommand(double x, double y, PathConstraints constraints, double endSpeed) {
        Command c = new Command() {
            Command pathCommand;

            {
                addRequirements(swerve);
            }

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
                        new GoalEndState(endSpeed, endPose.getRotation())
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
        return LoggedCommand.logCommand(c, "move +(" + x + "," + y + ")");
    }

}