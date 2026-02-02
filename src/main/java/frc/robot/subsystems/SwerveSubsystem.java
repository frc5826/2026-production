package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

public class SwerveSubsystem extends LoggedSubsystem {

    private SwerveDrive swerveDrive;
    private double feedForward = 5;

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
        setupPathPlanner();
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0,1));
    }

    private void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
        AutoBuilder.configure(
                this::getPose,
                null,
                () -> swerveDrive.getFieldVelocity(),
                this::drive,
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this
        );
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Command getLockCommand() {
        Command c = new RunCommand(swerveDrive::lockPose, this);
        return LoggedCommand.logCommand(c);

    }

    public void teleopDrive(double xSpeed, double ySpeed, double turnSpeed) {
        swerveDrive.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed));
    }

    public void drive(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }

    public double getMaxSpeed() {
        return feedForward;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }
}
// shelter = want; me have no shelter :(. Video games = need; me have video games :) food = want; me have no food :( stuff
