package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LoggedCommand;

public class IndexSubsystem extends LoggedSubsystem{

    private SparkMax innerIndexerMotor;
    private SparkMax outerIndexNConveyMotor;

//TODO Find SparkMax ID's
    public IndexSubsystem () {

        innerIndexerMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        outerIndexNConveyMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

    }

    public Command IndexCommand(double speed){

        Command command = new InstantCommand(()->{

            innerIndexerMotor.set(speed);
            outerIndexNConveyMotor.set(speed);

        }, this).finallyDo(()->{

            innerIndexerMotor.set(0);
            outerIndexNConveyMotor.set(0);

        });

        return LoggedCommand.logCommand(command);


    }

}
