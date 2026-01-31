// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;


public class RobotContainer {
    // public CameraSubsystem cameras = new CameraSubsystem();
    //public ShootSubsystem shoot = new ShootSubsystem();
    public SwerveSubsystem swerve = new SwerveSubsystem();
    public XboxController xbox = new XboxController(1);
     public IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {
        if (new File("/U/logs").isDirectory()) {
            DataLogManager.start("/U/logs");
        }
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        configureBindings();

        swerve.setDefaultCommand(new TeleopDriveCommand(swerve, xbox));

    }


    private void configureBindings() {
        new Trigger(()->xbox.getLeftTriggerAxis()>0.5).whileTrue(intake.getIntakeCommand(0.5));
        //new Trigger(()->xbox.getRightTriggerAxis()>0.5).whileTrue(shoot.getShootCommand(cameras::getHubDistance));
        new Trigger(xbox::getYButton).onTrue(new InstantCommand(swerve::zeroGyro));
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
//🧊🧊👶
//💍📴🔥
//ಠ╭╮ಠ
//🧔🔥🐔
//🐔🕺
//👨‍👩‍👧‍👦🧬💛🚢
//❌🛑🐝🚶‍♂️
//🏡🔛🙏
//IfYouReadThese...   ImSorry...
