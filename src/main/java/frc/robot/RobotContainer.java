// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.subsystems.*;

import java.io.File;


public class RobotContainer {

    public ShootSubsystem shoot = new ShootSubsystem();
    public SwerveSubsystem swerve = new SwerveSubsystem();
    public CameraSubsystem cameras = new CameraSubsystem(swerve::addVisionMeasurement);
    public XboxController xbox = new XboxController(1);
     public IntakeSubsystem intake = new IntakeSubsystem();
     public IndexSubsystem index = new IndexSubsystem();

    public RobotContainer() {
        if (new File("/U/logs").isDirectory()) {
            DataLogManager.start("/U/logs");
        }
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        configureBindings();

        //SmartDashboard.putData("shoot/fromDistance",shoot.getShootCommand(cameras::getHubDistance));
        //SmartDashboard.putData("shoot/stop",shoot.stopShoot());
//        swerve.setDefaultCommand(new TeleopDriveCommand(swerve, xbox));

    }


    private void configureBindings() {

        new Trigger(()->xbox.getLeftTriggerAxis()>0.5).whileTrue(intake.getIntakeCommand(0.5));
        new Trigger(()->xbox.getRightBumperButton()).onTrue(shoot.getShootCommand(cameras::getHubDistance));
        new Trigger(()->xbox.getRightTriggerAxis()>0.5).whileTrue(index.IndexCommand(0.3));
        new Trigger(()->xbox.getAButton()).toggleOnTrue(intake.getIntakeCommand(0.5));
        //new Trigger(()->xbox.getLeftTriggerAxis()>0.25).toggleOnTrue(end priority aim);
        //new Trigger(()->xbox.getYButton()).toggleOnTrue(climb);


    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}