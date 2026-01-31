package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex motor1;

    public IntakeSubsystem(){
        motor1=new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);


    }
    public Command getIntakeCommand(double speed){
        Command c=new RunCommand(()->setSpeed(speed),this).andThen(()->setSpeed(0));
        return LoggedCommand.logCommand(c);
    }
    public void setSpeed(double speed){
        motor1.set(speed);
    }
}
