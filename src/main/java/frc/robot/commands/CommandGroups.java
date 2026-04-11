package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
//    private ClimbSubsystem climb;
    private HoodSubsystem hood;
    private ConveyorSubsystem conveyor;
    private IntakeSubsystem intake;
    private ShootSubsystem shoot;
    private SwerveSubsystem swerve;
    private InnerIndexSubsystem innerIndex;
    private OuterIndexSubsystem outerIndex;

    private Command autoCommand;


    private String[] autoNames = new String[]{
            "shootOnly",
            "shallowMidAuto",
            "farMidAuto",
            "middleShootHumanShoot",
            "humanPlayerGrabShoot"
    };
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    public CommandGroups(CameraSubsystem camera, HoodSubsystem hood, ConveyorSubsystem conveyor, IntakeSubsystem intake, ShootSubsystem shoot, SwerveSubsystem swerve, InnerIndexSubsystem innerIndex, OuterIndexSubsystem outerIndex) {
        this.camera = camera;
//        this.climb = climb;
        this.hood = hood;
        this.conveyor = conveyor;
        this.intake = intake;
        this.shoot = shoot;
        this.swerve = swerve;
        this.innerIndex = innerIndex;
        this.outerIndex = outerIndex;

        for (String name : autoNames) {
            autoChooser.addOption(name, name);
        }
        autoChooser.setDefaultOption("empty", "empty");
        SmartDashboard.putString("5826/auto/current", "");
        SmartDashboard.putData("5826/auto/Chooser", autoChooser);
        SmartDashboard.putData("5826/auto/regenerate",
                new InstantCommand(() -> autoCommand = getAuto())
                        .ignoringDisable(true)
        );

        autoCommand = getAuto();

        autoChooser.onChange((x) -> autoCommand = getAuto());

    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public Command getAuto() {
        //Things that happen every time in auto go in init.
        //todo
        Command init = new InstantCommand().alongWith(intake.intakeDown());

        if (autoChooser.getSelected().equals("empty")) {
            SmartDashboard.putString("5826/auto/current", "Empty");
            return init;
        } else if (autoChooser.getSelected().equals("shootOnly")) {
            SmartDashboard.putString("5826/auto/current", "shootOnly");
            return init.andThen(moveCommand(-2, 0, cSlowPath, 0)).andThen(getAutoShootGroup());

        } else if (autoChooser.getSelected().equals("humanPlayerGrabShoot")) {
            SmartDashboard.putString("5826/auto/current", "humanPlayerGrabShoot");
            return init.alongWith((getPathCommand("humanOnlyAuto"))
                            .andThen(new WaitCommand(5))
                            .andThen(getPathCommand("humanOnlyAutoFinal"))
                            .alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getAutoShootGroup());

        } else if (autoChooser.getSelected().equals("farMidAuto")) {
            return init.alongWith(getMidAuto());

        } else if (autoChooser.getSelected().equals("shallowMidAuto")) {
            return init.alongWith(getShallowMidAuto());

        } else if (autoChooser.getSelected().equals("middleShootHumanShoot")) {
            return init.alongWith((getMirrorPathCommand("shallowMidAuto"))
                            .andThen(getMirrorPathCommand("shallowMidAutoFinal")).alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getAutoShootGroup().withTimeout(4))
                    .andThen(getPathCommand("humanFromRightAuto").deadlineFor(intake.getIntakeCommand()))
                    .andThen(new WaitCommand(2))
                    .andThen(getPathCommand("humanFromRightAutoFinal"))
                    .andThen(getAutoShootGroup());
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

    public Command getMirrorPathCommand(String commandName) {
        try {
            return LoggedCommand.logCommand(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(commandName).mirrorPath()),
                    "followPathCommand"
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public Command getMidAuto() {
        if (Locations.getLeftAutoZone().contains(swerve.getPose().getTranslation())) {
            SmartDashboard.putString("5826/auto/current", "LEFT DeepAuto");
            return (//First Pass
                    getPathCommand("midFromLeftAuto"))
                        .andThen(getPathCommand("leftFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                        .andThen(getAutoShootGroup().withTimeout(8))
                    //Second Pass
                    .andThen(getPathCommand("secondShallowMidAuto")
                            .andThen(getPathCommand("secondShallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                            .andThen(getAutoShootGroup().withTimeout(6))
                    );
        } else if (Locations.getRightAutoZone().contains(swerve.getPose().getTranslation())) {
            SmartDashboard.putString("5826/auto/current", "RIGHT DeepAuto");
            return (//First Pass
                    getMirrorPathCommand("midFromLeftAuto"))
                    .andThen(getMirrorPathCommand("leftFromMidAuto").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getAutoShootGroup().withTimeout(8))
                    //Second Pass
                    .andThen(getMirrorPathCommand("secondShallowMidAuto")
                            .andThen(getMirrorPathCommand("secondShallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                            .andThen(getAutoShootGroup().withTimeout(6))
                    );
        }


        return new InstantCommand();
    }

    public Command getShallowMidAuto() {
        if (Locations.getLeftAutoZone().contains(swerve.getPose().getTranslation())) {
            SmartDashboard.putString("5826/auto/current", "LEFT ShallowAuto");

            return (//First Pass
                    getPathCommand("shallowMidAuto"))
                    .andThen(getPathCommand("shallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getAutoShootGroup().withTimeout(5))
                    //Second Pass
                    .andThen(getPathCommand("secondShallowMidAuto")
                            .andThen(getPathCommand("secondShallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                            .andThen(getAutoShootGroup().withTimeout(4))
                    );
        } else if (Locations.getRightAutoZone().contains(swerve.getPose().getTranslation())) {
            SmartDashboard.putString("5826/auto/current", "RIGHT ShallowAuto");

            return (//First Pass
                    getMirrorPathCommand("shallowMidAuto"))
                    .andThen(getMirrorPathCommand("shallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                    .andThen(getAutoShootGroup().withTimeout(5))
                    //Second Pass
                    .andThen(getMirrorPathCommand("secondShallowMidAuto")
                            .andThen(getMirrorPathCommand("secondShallowMidAutoFinal").alongWith(shoot.getShootCommand(3100))).deadlineFor(intake.getIntakeCommand())
                            .andThen(getAutoShootGroup().withTimeout(4))
                    );
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

    public Command getAutoShootGroup() {
        return getSpinUpAim().andThen(Commands.parallel(
                new PriorityAimCommand(swerve),
                shoot.getShootCommand(swerve::getHubDistance, true),
                getInteyor().alongWith(intake.shakeIntakeCommand())
        )).finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen(Commands.parallel(
                new PriorityAimCommand(swerve),
                shoot.getShootCommand(swerve::getHubDistance, true),
                getInteyor().alongWith(intake.shakeIntakeCommand())
        )).finallyDo(shoot::stopShoot);
    }

    public Command getDumbShootGroup() {
        return shoot.getShootCommand(3100)
                .andThen(getInteyor())
                .alongWith(intake.shakeIntakeCommand())
                .finallyDo(shoot::stopShoot);
        //.until(shoot::isDoneShooting)
    }
    public Command getShuttleShoot() {
        return (shoot.getShootCommand(3000)
                .andThen(getInteyor())
                .alongWith(intake.shakeIntakeCommand())
                .finallyDo(shoot::stopShoot)).beforeStarting((() -> {
           if(Locations.getIsBlue()){
               swerve.setTurnGoal(Rotation2d.k180deg);
           } else{
               swerve.setTurnGoal(Rotation2d.kZero);
           }
        })).finallyDo(() -> swerve.endTurn());
        //.until(shoot::isDoneShooting)
    }

    public Command getDumbClimbShootGroup() {
        return shoot.getShootCommand(3600)
                .andThen(getInteyor())
                .alongWith(intake.shakeIntakeCommand())
                .finallyDo(shoot::stopShoot);
        //.until(shoot::isDoneShooting)
    }


    public Command getSpinUpAim() {
        return (shoot.getShootCommand(swerve::getHubDistance, true)
                .alongWith(
                        new PriorityAimCommand(swerve))
//                ).until(()->shoot.isAtGoalSpeed());
                ).until(() -> swerve.isAtTurnTarget() && shoot.isAtGoalSpeed());
    }

    public Command getInteyor() {
        return conveyor.getConveyorCommand().alongWith(innerIndex.getIndexCommand().alongWith(intake.getIntakeCommand())).alongWith(outerIndex.getIndexCommand());
    }

    public Command getDejammerCommand() {
        return intake.getReverseIntakeCommand().alongWith(innerIndex.getReverseIndexCommand()).alongWith(conveyor.getReverseConveyorCommand()).alongWith(outerIndex.getReverseIndexCommand());
    }

    public Command getWindUpCommand() {
        return innerIndex.getReverseIndexCommand().alongWith(conveyor.getReverseConveyorCommand().alongWith(outerIndex.getReverseIndexCommand()));
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