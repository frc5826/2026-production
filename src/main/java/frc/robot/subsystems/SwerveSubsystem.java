package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
//        swerveDrive.setVisionMeasurementStdDevs();

        turnController = new TurnController(0, 0, 0, 0, 0, 0, 0, () -> swerveDrive.getYaw().getRadians());

        SmartDashboard.putData("5826/auto/field",swerveDrive.field);
    }

    public void addVisionMeasurement(Pose2d robotPos, double timestamp ) {
        if(robotPos.getTranslation().getDistance(getPose().getTranslation())<1){
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.01,0.01,1000));

        }else if(DriverStation.isDisabled()){
            swerveDrive.addVisionMeasurement(robotPos, timestamp, VecBuilder.fill(0.01,0.01,0.1));

        }

    }

    //TODO Set turnController to control something
    public void setTurnGoal(Rotation2d targetAngle) {

        overrideTurn = true;
        turnController.setGoal(targetAngle.getRadians(), swerveDrive.getFieldVelocity().omegaRadiansPerSecond);

    }

    public void endTurn() {

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
            speeds.omegaRadiansPerSecond = turnController.calculate(0.02);
        }

        swerveDrive.driveFieldOriented(speeds);
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
