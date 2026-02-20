package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.TurnController;
import frc.robot.math.localization.Locations;
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
    private TurnController turnController;
    private boolean overrideTurn = false;

    private SwerveDrivePoseEstimator estimator;

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

        new SwerveDrivePoseEstimator(swerveDrive.kinematics,
                swerveDrive.getYaw(), swerveDrive.getModulePositions(), Pose2d.kZero,
                VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0));

        //TODO set turn controller coefficients
        turnController = new TurnController(1.3, 0.28, 4.5, 7.5, 0.7, 0, 0, () -> getPose().getRotation().getRadians());

        SmartDashboard.putData("5826/swerve/field", swerveDrive.field);
        SmartDashboard.putData("5826/swerve/turncontroller", turnController);
        SmartDashboard.putData("turnTo0", new InstantCommand(() -> setTurnGoal(Rotation2d.fromDegrees(0))));
        SmartDashboard.putData("turnTo90", new InstantCommand(() -> setTurnGoal(Rotation2d.fromDegrees(90))));
        SmartDashboard.putData("turnTo180", new InstantCommand(() -> setTurnGoal(Rotation2d.fromDegrees(180))));
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("5826/shoot/hubdistance", getHubDistance());
        SmartDashboard.putNumber("5826/swerve/turnSpeed", swerveDrive.getFieldVelocity().omegaRadiansPerSecond);
        SmartDashboard.putBoolean("5826/swerve/isAtTurnTarget", isAtTurnTarget());

    }

    public void addVisionMeasurement(Pose2d robotPos, double timestamp) {
        if (robotPos.getTranslation().getDistance(getPose().getTranslation()) < 1) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.01, 0.01, 0.1));

        } else if (DriverStation.isDisabled()) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.01, 0.01, 0.1));
            swerveDrive.setGyro(new Rotation3d(robotPos.getRotation()));

        }

    }

    public void setTurnGoal(Rotation2d targetAngle) {
        log("TurnOverride.init");
        overrideTurn = true;
        turnController.setGoal(targetAngle.getRadians(), -swerveDrive.getFieldVelocity().omegaRadiansPerSecond);

    }

    public void endTurn() {
        log("TurnOverride.end");
        overrideTurn = false;

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
        return LoggedCommand.logCommand(c, "Locked Command");

    }

    public void teleopDrive(double xSpeed, double ySpeed, double turnSpeed) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == DriverStation.Alliance.Red) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }
        drive(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed));
    }

    public void drive(ChassisSpeeds speeds) {

        if (overrideTurn) {
            speeds.omegaRadiansPerSecond = -turnController.calculate(0.02);
        }

        swerveDrive.drive(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, swerveDrive.getYaw()));
    }

    public double getMaxSpeed() {
        return feedForward;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public boolean isAtTurnTarget() {
        return turnController.isFinished();
    }

    public Command calibrationCommand() {
        Command c = new InstantCommand(() -> {
            swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0, 0, 0));
            swerveDrive.resetOdometry(Pose2d.kZero);
            swerveDrive.resetDriveEncoders();
        }).andThen(new RunCommand(() -> {
            swerveDrive.drive(new ChassisSpeeds(1, 0, 0));
            SmartDashboard.putNumber("Calibration/Speed", swerveDrive.getRobotVelocity().vxMetersPerSecond);
            SmartDashboard.putNumber("Calibration/Distance📡", swerveDrive.getPose().getX());
        }));
        return c;
    }

    public double getHubDistance() {
        return getPose().getTranslation().getDistance(Locations.getHubPose().getTranslation());
    }

//    {
//        "drive": {
//        "p": 0.0020645,
//                "i": 0,
//                "d": 0,
//                "f": 0.2,
//                "iz": 0
//    },
}
