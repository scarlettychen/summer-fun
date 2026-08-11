package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;

/**
 * Adapts an existing Ivy {@link Command} into an {@link Action} so it can be
 * passed into {@code new VerifiedCommand(action, verifier, ...)}. Use this
 * when the thing you want verified is already written as a normal command
 * (e.g. a whole path-following segment, or anything built with
 * {@code Commands.instant(...)} / {@code CommandBuilder}) — you don't need
 * to rewrite it against {@link Action} directly.
 *
 * This is pure delegation, nothing decided here: each {@link Action} method
 * calls straight through to the wrapped {@code Command}.
 */
public final class IvyCommandAction implements Action {

    private final Command command;

    public IvyCommandAction(Command command) {
        this.command = command;
    }

    @Override
    public void start() {
        command.start();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.done();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted ? EndCondition.INTERRUPTED : EndCondition.NATURALLY);
    }
}
