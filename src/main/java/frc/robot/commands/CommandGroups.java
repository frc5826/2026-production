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
            "runOutToMiddle+Shoot"
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
            return init.andThen(moveCommand(-2, 0,cSlowPath,0)).andThen(getShootGroup());

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
            if (Locations.getLeftAllianceZone().contains(swerve.getPose().getTranslation()))
                return getInteyor().alongWith(getPathCommand("toMiddleFromLeftPath").andThen(getShootGroup()));
            else if (Locations.getRightAllianceZone().contains(swerve.getPose().getTranslation())) {
                return getInteyor().alongWith(getPathCommand("toMiddleFromRightPath").andThen(getShootGroup()));
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

    public Command getPathDriveTestCommand() {
        return moveCommand(1, 0, cSlowPath,0).andThen(
                moveCommand(0,1,cSlowPath,0),
                moveCommand(-1,0,cSlowPath,0),
                moveCommand(0,-1,cSlowPath,0)
        );
    }

    public Command getPathTurnTestCommand() {
        return gotoCommand(new Pose2d(14,2, Rotation2d.kZero),cSlowPath,1)
                .andThen(gotoCommand(new Pose2d(14,4,Rotation2d.k180deg),cSlowPath,0));
    }

    public Command getShootGroup() {
        return getSpinUpAim().andThen(Commands.parallel(
                new PriorityAimCommand(swerve),
                shoot.getShootCommand(swerve::getHubDistance, true),
                getInteyor()
        )).finallyDo(shoot::stopShoot).until(shoot::isDoneShooting);
    }

    public Command getDumbShootGroup() {
        return shoot.getShootCommand(3000)
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

    public Command getDejammerCommand(){
        return intake.getReverseIntakeCommand().alongWith(conveyor.getReverseConveyorCommand()).withTimeout(1);
    }

    public Command gotoCommand(Pose2d endPose, PathConstraints constraints, double endSpeed){
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
                        constraints,
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
        return LoggedCommand.logCommand(c, "Drive to " + endPose);
    }

    public Command moveCommand(double x, double y, PathConstraints constraints, double endSpeed){
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
        return LoggedCommand.logCommand(c, "move +("+x+","+y+")");
    }

}