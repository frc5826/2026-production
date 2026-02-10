package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Index.*;

public class ConveyorSubsystem extends LoggedSubsystem {

    private SparkMax motor;

    public ConveyorSubsystem() {

        motor = new SparkMax(cOuterIndex, SparkLowLevel.MotorType.kBrushless);
    }

    public Command getConveyorCommand() {

        Command command = new RunCommand(() -> {

            motor.set(cConveyorSpeed);
        }, this).finallyDo(() -> {

            motor.set(0);
        });
        command.setName("Conveyor Command");
        return LoggedCommand.logCommand(command, "Conveyor Command");


    }

}
