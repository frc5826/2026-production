package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

import static frc.robot.Constants.Swerve.*;

import java.io.File;
import java.io.IOException;

public class SwerveSubsystem extends LoggedSubsystem {

    private SwerveDrive swerveDrive;
    private double feedForward = 5;
    private TurnController turnController;
    private boolean overrideTurn = false;

    private AHRS gyro;
    private Rotation2d gyroOffset = Rotation2d.kZero;
    private boolean speedsUpdated = false;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private boolean robotOriented;

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

        swerveDrive.setCosineCompensator(true);

        setupPathPlanner();

        gyro = (AHRS) swerveDrive.getGyro().getIMU();
        turnController = new TurnController(1.3, 0.28, 4.5, 7.5, 1, 0, 0, () -> getPose().getRotation().getRadians());

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        SmartDashboard.putData("5826/swerve/field", swerveDrive.field);
        SmartDashboard.putData("5826/swerve/turncontroller", turnController);
        SmartDashboard.putNumber("5826/swerve/ff/ks",swerveDrive.getModules()[0].getDefaultFeedforward().getKs());
        SmartDashboard.putNumber("5826/swerve/ff/kv",swerveDrive.getModules()[0].getDefaultFeedforward().getKv());
        SmartDashboard.putNumber("5826/swerve/ff/ka",swerveDrive.getModules()[0].getDefaultFeedforward().getKa());
    }

    @Override
    public void periodic() {
        super.periodic();
        if(DriverStation.isDisabled()){
            gyroOffset = getPose().getRotation().minus(Rotation2d.fromDegrees(-gyro.getAngle()));
        }
        if(speedsUpdated){
            speedsUpdated = false;
        } else {
            speeds = cStopped;
        }
        drive(speeds, robotOriented);

        SmartDashboard.putNumber("5826/shoot/hubdistance", getHubDistance());
        SmartDashboard.putNumber("5826/swerve/gyroAngle", Rotation2d.fromDegrees(-gyro.getAngle()).plus(gyroOffset).getDegrees());
        SmartDashboard.putNumber("5826/swerve/turnSpeed", swerveDrive.getFieldVelocity().omegaRadiansPerSecond);
        SmartDashboard.putBoolean("5826/swerve/isAtTurnTarget", isAtTurnTarget());

    }

    public void addVisionMeasurement(Pose2d robotPos, double timestamp, Matrix<N3, N1> stdDevs) {
        if (robotPos.getTranslation().getDistance(getPose().getTranslation()) < 1) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, stdDevs);

        } else if (DriverStation.isDisabled()) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.05, 0.05, 0.5));

        }

    }

    public void setTurnGoal(Rotation2d targetAngle) {
        log("TurnOverride.init");
        overrideTurn = true;
        turnController.setGoal(targetAngle.getRadians(), swerveDrive.getFieldVelocity().omegaRadiansPerSecond);

    }

    public void endTurn() {
        log("TurnOverride.end");
        overrideTurn = false;

    }

    public void resetDriveEncoders() {
        swerveDrive.resetDriveEncoders();
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
                (x) -> setSpeeds(x, true),
                new PPHolonomicDriveController(
                        cPathDrivePID,
                        cPathTurnPID
                ),
                config,
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this
        );
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Command getLockCommand() {
        Command c = new RunCommand(swerveDrive::lockPose, this);
        return LoggedCommand.logCommand(c, "Locked Command");

    }

    public void teleopDrive(double xSpeed, double ySpeed, double turnSpeed) {
        if (!Locations.getIsBlue()) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }
        setSpeeds(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed), false);
    }

    public void drive(ChassisSpeeds speeds, boolean robotOriented) {

        if (overrideTurn) {
            speeds.omegaRadiansPerSecond = turnController.calculate(0.02);
        }

        if(robotOriented){
            swerveDrive.drive(speeds);
        } else {
            swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(-gyro.getAngle()).plus(gyroOffset)));
        }
    }

    public void setSpeeds (ChassisSpeeds speeds, boolean robotOriented){
        this.speeds = speeds;
        this.robotOriented = robotOriented;
        speedsUpdated = true;
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
