package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends LoggedSubsystem {
    private SparkFlex intakeMotor, intakeMotorFollower;
    private SparkMax armMotor;
    private SparkMax armMotorFollower;
    private Subsystem armSubsystem = new LoggedSubsystem();
    private double armSetpoint;
    private boolean armMovingUp = true;
    private boolean isIntakeRunning = false;


    public IntakeSubsystem() {
        intakeMotor = new SparkFlex(cMotorIDIntake1, SparkLowLevel.MotorType.kBrushless);
        intakeMotorFollower = new SparkFlex(cMotorIDIntake2, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig intakeConfig = new SparkFlexConfig().inverted(true).smartCurrentLimit(60).idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotorFollower.configure(intakeConfig.follow(intakeMotor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor = new SparkMax(cArmMotor, SparkLowLevel.MotorType.kBrushless);
        armMotorFollower = new SparkMax(cArmMotorFollower, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig().smartCurrentLimit(20)
                .apply(new ClosedLoopConfig().pid(intakeP, intakeI, intakeD).outputRange(-0.2, 0.2))
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .apply(new EncoderConfig().positionConversionFactor(3 * 5 * 5 * 2 * 6 / 360.0));
        armMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorFollower.configure(config.follow(armMotor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Preferences.initBoolean("IntakeUpDown", true);

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("5826/Intake/ArmPosition", armMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("5826/Intake/IntakeRunning", intakeRunning());
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
        //TODO double equals could be tricky (0.000000001)
        isIntakeRunning = speed != 0.0;
    }

    public Command shakeIntakeCommand() {
        Command c = new InstantCommand(() -> armSetpoint = 0).
                andThen(new RunCommand(
                        () -> {
//                    double currentPosition = armMotor.getClosedLoopController().getSetpoint();
                            if (armSetpoint < cMaxIntakeShake) {
                                armMovingUp = false;
                            } else if (armSetpoint > cMinIntakeShake) {
                                armMovingUp = true;
                            }

                            if (armMovingUp) {
                                armSetpoint -= cShakeSpeed;
                                armMotor.getClosedLoopController().setSetpoint(armSetpoint, SparkBase.ControlType.kPosition);
                            } else if (!armMovingUp) {
                                armSetpoint += cShakeSpeed;
                                armMotor.getClosedLoopController().setSetpoint(armSetpoint, SparkBase.ControlType.kPosition);
                            }
                        }
                )).finallyDo(() -> {
                    armMotor.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                }).onlyIf(() -> Preferences.getBoolean("IntakeUpDown", true));
        return LoggedCommand.logCommand(c, "Intake Shake Command");
    }

    ;


    public Command intakeDown() {
//        Command c = new InstantCommand(()-> armMotor.getEncoder().setPosition(0))
//          .andThen(new RunCommand(() -> {
//              armMotor.getClosedLoopController().setSetpoint(-160, SparkBase.ControlType.kPosition);
//          }, armSubsystem).withTimeout(1.3));
//        return LoggedCommand.logCommand(c, "Intake Down");

        Command c = new RunCommand(() -> {
            armMotor.set(cArmMotorSpeed);
        }, armSubsystem).withTimeout(1.3)
                .finallyDo(() -> {
                    armMotor.getClosedLoopController().setSetpoint(0, SparkBase.ControlType.kPosition);
                    armMotor.getEncoder().setPosition(0);
                });
        return LoggedCommand.logCommand(c, "Intake Down");
    }

    private boolean intakeRunning () {
       return isIntakeRunning;
    }

}
