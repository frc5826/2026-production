package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex intakeMotor;
    private SparkMax armMotor;
    private SparkMax armMotorFollower;
    private Subsystem armSubsystem = new LoggedSubsystem();
    private boolean armMovingUp = true;

    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(cMotorIDIntake1, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig intakeConfig = new SparkFlexConfig().inverted(true).smartCurrentLimit(40);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor = new SparkMax(cArmMotor, SparkLowLevel.MotorType.kBrushless);
        armMotorFollower = new SparkMax(cArmMotorFollower, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20)
                .apply(new ClosedLoopConfig().pid(intakeP, intakeI, intakeD).outputRange(-0.2, 0.2))
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .apply(new EncoderConfig().positionConversionFactor(3 * 5 * 5 * 2 / 360.0));
        armMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorFollower.configure(config.follow(armMotor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }


    public void setIntakePositionDegrees(double angle) {
        armMotor.getClosedLoopController().setSetpoint(angle, SparkBase.ControlType.kPosition);
    }

    public void setIntakePositionRadians(double angle) {
        setIntakePositionDegrees(Math.toDegrees(angle));
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

    public Command getReverseIntakeCommand() {
        Command c = new RunCommand(() -> setSpeed(-cSpeed), this).finallyDo(() -> setSpeed(0));
        return LoggedCommand.logCommand(c, "Reverse Intake Command");
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command moveIntake() {
        Command c = new RunCommand(
                () -> {
                    setIntakePositionDegrees(45);
                }
                , armSubsystem);
        return c;
    }

    public Command shakeIntakeCommand() {
        Command c = new RunCommand(
                () -> {
                    double currentPosition = armMotor.getClosedLoopController().getSetpoint();
                    if (currentPosition > cMaxIntakeShake) {
                        armMovingUp = false;
                    } else if (currentPosition < cMinIntakeShake) {
                        armMovingUp = true;
                    }

                    if (armMovingUp) {
                        currentPosition+=0.05;
                        armMotor.getClosedLoopController().setSetpoint(currentPosition, SparkBase.ControlType.kPosition);
                    } else if (!armMovingUp) {
                        currentPosition-=0.05;
                        armMotor.getClosedLoopController().setSetpoint(currentPosition, SparkBase.ControlType.kPosition);
                    }
                }, armSubsystem
        ).finallyDo(() -> {
            armMotor.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
        });
        return LoggedCommand.logCommand(c,"Intake Shake Command");
    }

    ;


    public Command intakeDown() {

        Command c = new RunCommand(() -> {
            armMotor.set(cArmMotorSpeed);
            //  armMotorFollower.set(cArmMotorSpeed);
        }, armSubsystem).withTimeout(2)
                .finallyDo(() -> {
                    armMotor.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                    armMotor.getEncoder().setPosition(0);
                    //     armMotorFollower.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                    //     armMotorFollower.getEncoder().setPosition(0);
                });
        return LoggedCommand.logCommand(c, "Intake Down");
    }

}
