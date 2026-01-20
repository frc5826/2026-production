package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LoggedCommand;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;

public class SwerveSubsystem extends LoggedSubsystem {

    private SwerveDrive swerveDrive;
    private double feedForward=0;//todo

    public SwerveSubsystem() {

        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(4), 6.12, 1);

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory() + "/swerve")).createSwerveDrive(feedForward, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
    public Command getLockCommand(){
        Command c = new RunCommand(swerveDrive::lockPose,this);
        return LoggedCommand.logCommand(c);
    }
}