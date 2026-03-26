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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.util.DoubleCircularBuffer;
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
    CircularBuffer<Translation2d> recentCameraPoses = new CircularBuffer<>(3);

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
        //TODO Tune to be Faster!!
        turnController = new TurnController(1.3, 0.28, 4.5, 7.5, 0.7, 0.0, 0.0, () -> getPose().getRotation().getRadians());

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        SmartDashboard.putData("5826/swerve/field", swerveDrive.field);
        SmartDashboard.putData("5826/swerve/turncontroller", turnController);
        SmartDashboard.putData("5826/swerve/resetAngleEncoders", new InstantCommand(()->swerveDrive.synchronizeModuleEncoders()).ignoringDisable(true));

        swerveDrive.setModuleEncoderAutoSynchronize(true, 3);
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

//        SmartDashboard.putNumber("5826/shoot/hubdistance", getHubDistance());
//        SmartDashboard.putBoolean("5826/swerve/isAtTurnTarget", isAtTurnTarget());
    }

    public void addVisionMeasurement(Pose2d robotPos, double timestamp, Matrix<N3, N1> stdDevs) {
        if (DriverStation.isDisabled()) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.1, 0.1, 1));

        } else if (!Locations.getFieldZone().contains(getPose().getTranslation())) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0, 0, 0));
        } else if (robotPos.getTranslation().getDistance(getPose().getTranslation()) < 5) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, stdDevs);
        }
    }
        /*TODO investigate this
        } else if (robotPos.getTranslation().getDistance(getAverageTranslation(recentCameraPoses)) < 5) {
            swerveDrive.addVisionMeasurement(robotPos, timestamp, stdDevs);
        }
        recentCameraPoses.addFirst(robotPos.getTranslation());
    }


    private Translation2d getAverageTranslation(CircularBuffer<Translation2d> buffer) {
        double x = 0,y = 0;
        for (int i = 0; i < buffer.size(); i++) {
            Translation2d t = buffer.get(i);
            x+=t.getX();
            y+=t.getY();
        }
        x /= buffer.size();
        y /= buffer.size();
        return new Translation2d(x,y);
    }*/

    public void setTurnGoal(Rotation2d targetAngle) {
        if (!overrideTurn) {
            log("TurnOverride.init");
        }
        overrideTurn = true;
        turnController.setGoal(targetAngle.getRadians());

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
