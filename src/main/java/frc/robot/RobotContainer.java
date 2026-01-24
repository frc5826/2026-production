// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.io.File;


public class RobotContainer
{
   // public CameraSubsystem cameras = new CameraSubsystem();
   public ShootSubsystem shoot = new ShootSubsystem();
    public RobotContainer()

    {
        if (new File("/U/logs").isDirectory()){
            DataLogManager.start("/U/logs");
        }
        SignalLogger.enableAutoLogging(false);

        configureBindings();

    }
    
    
    private void configureBindings() {}
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
