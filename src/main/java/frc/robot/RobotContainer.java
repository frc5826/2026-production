// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
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
import frc.robot.commands.swerve.PriorityAimCommand;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.subsystems.*;

import java.io.File;


public class RobotContainer {

    public ShootSubsystem shoot = new ShootSubsystem();
    public SwerveSubsystem swerve = new SwerveSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();
    public CameraSubsystem cameras = new CameraSubsystem(swerve::addVisionMeasurement);
    public IntakeSubsystem intake = new IntakeSubsystem();
    public ConveyorSubsystem conveyor = new ConveyorSubsystem();
    public IndexSubsystem index = new IndexSubsystem();
    public SensorSubsystem sensor = new SensorSubsystem();
    public PriorityAimCommand priority = new PriorityAimCommand(swerve, cameras);
    public PowerDistribution pdp = new PowerDistribution(50, PowerDistribution.ModuleType.kRev);

    public XboxController xbox = new XboxController(1);

    public CommandGroups commandGroups = new CommandGroups(cameras, climb, hood, conveyor, intake, shoot, swerve, index, sensor);

    public RobotContainer() {
        if (new File("/U/logs").isDirectory()) {
            DataLogManager.start("/U/logs");
            DriverStation.startDataLog(DataLogManager.getLog());
        }
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        configureBindings();

        swerve.setDefaultCommand(new TeleopDriveCommand(swerve,xbox));

        SmartDashboard.putData("5826/powerdist", pdp);

    }


    private void configureBindings() {

//        new Trigger(() -> xbox.getRightBumperButton()).onTrue(commandGroups.getSpinUpAim());
//        new Trigger(() -> xbox.getRightBumperButton()).onTrue(commandGroups.getSpinUpAim());
        new Trigger(() -> xbox.getRightTriggerAxis() > 0.5).whileTrue(commandGroups.getShootGroup());
        new Trigger(() -> xbox.getAButton()).toggleOnTrue(intake.getIntakeCommand());
        new Trigger(() -> xbox.getLeftTriggerAxis() > 0.5).toggleOnTrue(priority);
        new Trigger(() -> xbox.getRightBumperButton()).whileTrue(commandGroups.getDumbShootGroup());
//        new Trigger(() -> xbox.getLeftBumperButton()).toggleOnTrue(commandGroups.getSpinUpAim());
//        new Trigger(() -> xbox.getXButton()).onTrue(intake.intakeDown());
        new Trigger(() -> xbox.getBButton()).whileTrue(conveyor.getConveyorCommand());
        new Trigger(() -> xbox.getBackButton()).onTrue(new InstantCommand(swerve::zeroGyro));

        new Trigger(() -> xbox.getStartButton()).whileTrue(index.getIndexCommand());
        //new Trigger(() -> xbox.getRightTriggerAxis()> 0.5).whileTrue(commandGroups.getDumbShootGroup());
        //
        //new Trigger(()->xbox.getLeftTriggerAxis()>0.25).toggleOnTrue(end priority aim);
        //new Trigger(()->xbox.getYButton()).toggleOnTrue(climb);


    }


    public Command getAutonomousCommand() {
        return commandGroups.getAuto();
    }
}