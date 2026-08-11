package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import java.util.Arrays;
import java.util.Collections;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Adapts any {@link Action} — typically a finished {@link VerifiedCommand} —
 * into an Ivy {@link Command} so it can be scheduled directly with
 * {@code Scheduler}/{@code Groups.sequential/parallel/race} next to ordinary
 * commands in {@code OpmodeCommands}. This is the other direction from
 * {@link IvyCommandAction}.
 *
 * Requirements/priority/behaviors aren't part of {@link Action} (they're an
 * Ivy-specific scheduling concern), so they're supplied here at the
 * integration boundary instead of leaking into the neutral Action contract.
 * Defaults match {@code CommandBuilder}'s defaults (no requirements,
 * priority 0, end-on-interrupt, cancel-on-block, override-on-conflict) —
 * pass requirements or use the other constructor if the wrapped action
 * actually owns hardware.
 */
public final class IvyCommandAdapter implements Command {

    private final Action action;
    private final Set<Object> requirements;
    private final int priority;
    private final InterruptedBehavior interruptedBehavior;
    private final BlockedBehavior blockedBehavior;
    private final ConflictBehavior conflictBehavior;

    public IvyCommandAdapter(Action action, Object... requirements) {
        this(
                action,
                Arrays.stream(requirements).collect(Collectors.toSet()),
                0,
                InterruptedBehavior.END,
                BlockedBehavior.CANCEL,
                ConflictBehavior.OVERRIDE);
    }

    public IvyCommandAdapter(
            Action action,
            Set<Object> requirements,
            int priority,
            InterruptedBehavior interruptedBehavior,
            BlockedBehavior blockedBehavior,
            ConflictBehavior conflictBehavior) {
        this.action = action;
        this.requirements = requirements == null ? Collections.emptySet() : requirements;
        this.priority = priority;
        this.interruptedBehavior = interruptedBehavior;
        this.blockedBehavior = blockedBehavior;
        this.conflictBehavior = conflictBehavior;
    }

    @Override
    public Set<Object> requirements() {
        return requirements;
    }

    @Override
    public int priority() {
        return priority;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return interruptedBehavior;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return conflictBehavior;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return blockedBehavior;
    }

    @Override
    public void start() {
        action.start();
    }

    @Override
    public boolean done() {
        return action.isFinished();
    }

    @Override
    public void execute() {
        action.execute();
    }

    @Override
    public void end(EndCondition endCondition) {
        action.end(endCondition != EndCondition.NATURALLY);
    }
}
