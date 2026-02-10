package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LoggedCommand;

public class ClimbSubsystem extends LoggedSubsystem {

    public ClimbSubsystem (){


    }
    public void climb () {
        //todo

    }

    public void down () {
        //todo

    }
    public Command climbCommand () {
        Command c = new InstantCommand(this::climb,this);
        return LoggedCommand.logCommand(c,"Climb Command");
    }

    public Command downCommand () {
        Command c = new InstantCommand(this::down, this);
        return LoggedCommand.logCommand(c, "Down Command");
    }
}




