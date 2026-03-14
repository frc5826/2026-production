// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.swerve.PathToFromMid;
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.math.HubWidget;
import frc.robot.subsystems.*;

import java.io.File;
import java.sql.Driver;


public class RobotContainer {

    public ShootSubsystem shoot = new ShootSubsystem();
    public SwerveSubsystem swerve = new SwerveSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();
    public CameraSubsystem cameras = new CameraSubsystem(swerve::addVisionMeasurement);
    public IntakeSubsystem intake = new IntakeSubsystem();
    public ConveyorSubsystem conveyor = new ConveyorSubsystem();
    public IndexSubsystem index = new IndexSubsystem();
    public PowerDistribution pdp = new PowerDistribution(50, PowerDistribution.ModuleType.kRev);

    public XboxController xbox = new XboxController(1);

    public CommandGroups commandGroups = new CommandGroups(cameras, climb, hood, conveyor, intake, shoot, swerve, index);

    public RobotContainer() {
        if (new File("/U/logs").isDirectory()) {
            DataLogManager.start("/U/logs");
            DriverStation.startDataLog(DataLogManager.getLog());
        }
        UsbCamera driverView = CameraServer.startAutomaticCapture();
        driverView.setFPS(7);
        driverView.setExposureManual(8);
        driverView.setBrightness(35);
        driverView.setResolution(400,300);
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        configureBindings();

        swerve.setDefaultCommand(new TeleopDriveCommand(swerve,xbox));

        SmartDashboard.putData("5826/powerdist", pdp);
        SmartDashboard.putData("5826/Hub",new HubWidget());
    }


    private void configureBindings() {
        /* Triggers */
        new Trigger(() -> xbox.getRightTriggerAxis() > 0.5).whileTrue(commandGroups.getShootGroup());
        new Trigger(() -> xbox.getLeftTriggerAxis() > 0.5).toggleOnTrue(new PriorityAimCommand(swerve));

        /* Bumpers */
        //new Trigger(() -> xbox.getLeftBumperButton())
        new Trigger(() -> xbox.getRightBumperButton()).whileTrue(commandGroups.getDumbShootGroup());


        /* A B X Y */
        new Trigger(() -> xbox.getAButton()).toggleOnTrue(intake.getIntakeCommand());
        new Trigger(()-> xbox.getBButton()).whileTrue(PathToFromMid.get(swerve));
        new Trigger(()-> xbox.getYButton()).onTrue(commandGroups.getPathTurnTestCommand());
        new Trigger(() -> xbox.getXButton()).onTrue(commandGroups.getDejammerCommand());

        /* Menu Buttons */
        new Trigger(() -> xbox.getStartButton()).whileTrue(index.getIndexCommand());
        new Trigger(() -> xbox.getBackButton()).onTrue(new InstantCommand(swerve::zeroGyro));

        /* D - Pad */
//        new Trigger(()-> xbox.getPOV()==0).whileTrue(climb.downCommand());
//        new Trigger(()-> xbox.getPOV()== 90).whileTrue(climb.climbCommand());
//        new Trigger(()-> xbox.getPOV()== 180).whileTrue(climb.stowCommand());
        //new Trigger(() - > xbox.getPOV() == 270))

    }



    public Command getAutonomousCommand() {
        return commandGroups.getAuto();
    }
}
