package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Climb.*;

public class ClimbSubsystem extends LoggedSubsystem {

    private SparkMax motor;
    private double lastPos;

    public ClimbSubsystem (){

        motor = new SparkMax(cClimber, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig()
                .apply(new EncoderConfig().positionConversionFactor(cConfigMultiplier))
                .apply(new ClosedLoopConfig().pid(5, 0, 0).outputRange(-0.5,1));
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zeroClimb();
        hookStow();
        SmartDashboard.putData("5826/climb/zeroClimber", zeroCommand());
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("5826/climb/position", motor.getEncoder().getPosition());
    }

    public void hookDown() {
        if(motor.getEncoder().getPosition()<=cDownPos*1.5){
            motor.getClosedLoopController().setSetpoint(cDownPos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);

        }else
            motor.getClosedLoopController().setSetpoint(cDownPos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, -6);
    }

    public void hookStow() {
        if(motor.getEncoder().getPosition()<=cDownPos*1.5){
            motor.getClosedLoopController().setSetpoint(cStowPos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
        }else
            motor.getClosedLoopController().setSetpoint(cStowPos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, -6);
    }

    public void hookUp() {
        motor.getClosedLoopController().setSetpoint(cUpPos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void zeroClimb () {
        motor.getEncoder().setPosition(0);
    }

    public Command climbCommand () {
        Command c = new RunCommand(this::hookDown,this);
        return LoggedCommand.logCommand(c,"Climb Command");
    }

    public Command downCommand () {
        Command c = new RunCommand(this::hookUp, this);
        return LoggedCommand.logCommand(c, "Down Command");
    }

    public Command stowCommand () {
        Command c = new RunCommand(this::hookStow, this);
        return LoggedCommand.logCommand(c, "Down Command");
    }

    public Command zeroCommand() {
        return new RunCommand(() -> {
            motor.set(-0.2);
        }, this)
                .finallyDo(()-> {
                    zeroClimb();
                    hookStow();
                });
    }
}




