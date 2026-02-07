package frc.robot.commands;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.LoggedSubsystem;

import java.util.Set;
import java.util.function.Consumer;

public class LoggedCommand extends Command {
    private Command command = new InstantCommand(()->{});
    private LoggedCommand (Command command  ) {
        this.command = command;
        CommandScheduler.getInstance().registerComposedCommands(command);
        setName(command.getName());
    }
    public LoggedCommand () {
        command = this;
    }

    public static Command logCommand(Command command) {
        return new LoggedCommand(command);
    }

    @Override
    public void initialize() {
        for (Subsystem s:getRequirements()) {
            if (s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).Log(getName()+".initialize");
            }
        }
        if (command != this)
            command.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        if (command != this)
            command.execute();
    }

    @Override
    public boolean isFinished() {
        if (command != this)
            return command.isFinished();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        for (Subsystem s:getRequirements()) {
            if (s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).Log(getName()+".end");
            }
        }
        if (command != this)
            command.end(interrupted);

    }

    @Override
    public Set<Subsystem> getRequirements() {
        if (command != this)
            return command.getRequirements();
        return super.getRequirements();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (command != this)
            return command.getInterruptionBehavior();
        return super.getInterruptionBehavior();
    }

    @Override
    public boolean runsWhenDisabled() {
        if (command != this)
            return command.runsWhenDisabled();
        else return super.runsWhenDisabled();
    }
}


