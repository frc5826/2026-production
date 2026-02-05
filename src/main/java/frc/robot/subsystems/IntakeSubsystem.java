package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex intakeMotor;
    private SparkMax armMotor;

    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(5, SparkLowLevel.MotorType.kBrushless);


    }

    public Command getIntakeCommand(double speed) {
        Command c = new RunCommand(() -> setSpeed(speed), this).finallyDo(() -> setSpeed(0));
        return LoggedCommand.logCommand(c);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command intakeDown(double angle) {
        Command c = new RunCommand(() -> armMotor.set(0.5), this).withTimeout(1).finallyDo(() -> armMotor.set(0));
        return LoggedCommand.logCommand(c);
    }

}
