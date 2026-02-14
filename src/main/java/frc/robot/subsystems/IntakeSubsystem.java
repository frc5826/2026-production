package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex intakeMotor;
    private SparkMax armMotor;
    private SparkMax armMotorFollower;

    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(cMotorIDIntake1, SparkLowLevel.MotorType.kBrushless);
        intakeMotor.configure(new SparkFlexConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor = new SparkMax(cArmMotor, SparkLowLevel.MotorType.kBrushless);
        armMotorFollower = new SparkMax(cArmMotorFollower, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20).apply(new ClosedLoopConfig().p(0.2).outputRange(-0.2,0.2));
        armMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorFollower.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("5826/Intake/ArmPosition", armMotor.getEncoder().getPosition());
    }

    public Command getIntakeCommand() {
        Command c = new RunCommand(() -> setSpeed(cSpeed), this).finallyDo(() -> setSpeed(0));
        return LoggedCommand.logCommand(c, "Intake Command");
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command intakeDown() {
        Subsystem subsystem = new LoggedSubsystem();
        Command c = new RunCommand(() -> {
            armMotor.set(cArmMotorSpeed);
            armMotorFollower.set(cArmMotorSpeed);
        }, subsystem).withTimeout(2)
                .finallyDo(() -> {
                    armMotor.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                    armMotor.getEncoder().setPosition(0);
                    armMotorFollower.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                    armMotorFollower.getEncoder().setPosition(0);
                });
        return LoggedCommand.logCommand(c, "Intake Down");
    }

}
