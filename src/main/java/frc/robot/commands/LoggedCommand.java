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

    }
    public static Command logCommand(Command command) {
        return new LoggedCommand(command);
    }

    @Override
    public void initialize() {
        super.initialize();
        for (Subsystem s:getRequirements()) {
            if (s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).Log(getName()+".initialize");
            }
        }
        command.initialize();
    }


    @Override
    public void execute() {
        super.execute();
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        for (Subsystem s:getRequirements()) {
            if (s instanceof LoggedSubsystem) {
                ((LoggedSubsystem) s).Log(getName()+".end");
            }

        }
        command.end(interrupted);

    }

    @Override
    public Set<Subsystem> getRequirements() {
        return command.getRequirements();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();

    }

    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }
}


