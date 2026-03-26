package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Index.*;


public class OuterIndexSubsystem extends LoggedSubsystem {

    private SparkFlex motor;

    public OuterIndexSubsystem() {

        motor = new SparkFlex(cIndex, SparkLowLevel.MotorType.kBrushless);
        motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command getIndexCommand() {

        Command command = new RunCommand(() -> {

            motor.set(cOuterIndexSpeed);
        }, this).finallyDo(() -> {

            motor.set(0);
        });

        return LoggedCommand.logCommand(command, "Outer Index Command");


    }

    public Command getReverseIndexCommand() {

        Command command = new RunCommand(() -> {

            motor.set(-cOuterIndexSpeed);
        }, this).finallyDo(() -> {

            motor.set(0);
        });

        return LoggedCommand.logCommand(command, "Reverse Outer Index Command");


    }
}


