package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.localization.Locations;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.*;

public class PathToFromMid extends Command{

    private SwerveSubsystem swerve;
    private Command pathCommand;

    public static Command get(SwerveSubsystem swerve) {
        return LoggedCommand.logCommand(new PathToFromMid(swerve), "PathToFromMid");
    }

    private PathToFromMid(SwerveSubsystem swerve){

        this.swerve = swerve;
        addRequirements(swerve);

    }

    @Override
    public void initialize() {
        super.initialize();
            if (Locations.getRightAllianceZonePose().contains(swerve.getPose().getTranslation())){
                pathCommand = getPathCommand("midFromRight");
            } else if (Locations.getLeftAllianceZonePose().contains(swerve.getPose().getTranslation())) {
                pathCommand = getPathCommand("midFromLeft");
            } else if (Locations.getLeftSideMidPose().contains(swerve.getPose().getTranslation())) {
                pathCommand = getPathCommand("leftFromMid");
            } else if (Locations.getRightSideMidPose().contains(swerve.getPose().getTranslation())) {
                pathCommand = getPathCommand("rightFromMid");
            }
            else {
                pathCommand = new PrintCommand("Invalid Position");
            }


    }

    @Override
    public void execute() {
        super.execute();
        pathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        pathCommand.end(interrupted);
    }


    private Command getPathCommand(String name) {
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(name), cSlowPath);
        }
        catch (Exception e) {
            e.printStackTrace();
            return new InstantCommand();
        }

    }
}
