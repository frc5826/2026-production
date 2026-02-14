package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Index.*;

public class IndexSubsystem extends LoggedSubsystem {

    private SparkMax motor;

    public IndexSubsystem() {

        motor = new SparkMax(cInnerIndex, SparkLowLevel.MotorType.kBrushless);
        motor.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command getIndexCommand() {

        Command command = new RunCommand(() -> {

            motor.set(cIndexerSpeed);
        }, this).finallyDo(() -> {

            motor.set(0);
        });

        return LoggedCommand.logCommand(command, "Index Command");


    }

}
