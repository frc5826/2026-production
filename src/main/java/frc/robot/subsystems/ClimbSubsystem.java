package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LoggedCommand;

import static frc.robot.Constants.Climb.*;

public class ClimbSubsystem extends LoggedSubsystem {

    private SparkMax motor;


    public ClimbSubsystem (){

//        motor = new SparkMax(cClimber, SparkLowLevel.MotorType.kBrushless);
//        SparkBaseConfig config = new SparkMaxConfig()
//                .apply(new EncoderConfig().positionConversionFactor(cConfigMultiplier))
//                .apply(new ClosedLoopConfig().pid(0, 0, 0).outputRange(-1,1));
//        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        super.periodic();
//        SmartDashboard.putNumber("5826/climb/position", motor.getEncoder().getPosition());
    }

//    public void hookDown() {
//        motor.getClosedLoopController().setSetpoint(cDownPos, SparkBase.ControlType.kPosition);
//    }
//
//    public void hookUp() {
//        motor.getClosedLoopController().setSetpoint(cUpPos, SparkBase.ControlType.kPosition);
//    }
//
//    public void zeroClimb () {
//        motor.getEncoder().setPosition(0);
//    }

//    public Command climbCommand () {
////        Command c = new InstantCommand(this::hookDown,this);
////        return LoggedCommand.logCommand(c,"Climb Command");
//        return null;
//    }
//
//    public Command downCommand () {
////        Command c = new InstantCommand(this::hookUp, this);
////        return LoggedCommand.logCommand(c, "Down Command");
//        return null;
//    }
}




