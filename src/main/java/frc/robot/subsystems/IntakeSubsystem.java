package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex intakeMotor;
    private SparkMax armMotor;
    private SparkMax armMotorFollower;

    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(cMotorIDIntake1, SparkLowLevel.MotorType.kBrushless);
        armMotor = new SparkMax(cArmMotor, SparkLowLevel.MotorType.kBrushless);
        armMotorFollower = new SparkMax(cArmMotorFollower, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20);
        armMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorFollower.configure(config.follow(armMotor,true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command getIntakeCommand(double speed) {
        Command c = new RunCommand(() -> setSpeed(speed), this).finallyDo(() -> setSpeed(0));
        return LoggedCommand.logCommand(c);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command intakeDown() {
        Command c = new RunCommand(() -> armMotor.set(cArmMotorSpeed), this).withTimeout(1).finallyDo(() -> armMotor.set(0));
        return LoggedCommand.logCommand(c);
    }

}
