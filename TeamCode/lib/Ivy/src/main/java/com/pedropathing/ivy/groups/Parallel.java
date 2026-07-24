package com.pedropathing.ivy.groups;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * A command group that runs multiple commands in parallel.
 *
 * @author Baron Henderson
 * @author Kabir Goyal
 * @version 1.0
 */
class Parallel extends CommandBuilder {
    private final List<Command> children;
    private final Map<Command, Boolean> finished = new HashMap<>();

    /**
     * Constructs a new Parallel command group with the passed in commands
     *
     * @param children the commands to run in parallel
     */
    public Parallel(Command... children) {
        this.children = new ArrayList<>(Arrays.asList(children));
        for (Command command : this.children) {
            finished.put(command, false);
        }

        requiring(
                this.children.stream()
                        .flatMap(command -> command.requirements().stream())
                        .collect(Collectors.toSet())
        );

        setPriority(this.children.stream().mapToInt(Command::priority).max().orElse(0));

        setExecute(() -> {
            if (done()) return;

            for (Command command : this.children) {
                if (Boolean.TRUE.equals(finished.get(command))) {
                    continue;
                }

                if (command.done()) {
                    command.end(EndCondition.NATURALLY);
                    finished.put(command, true);
                    continue;
                }

                command.execute();
            }
        });

        setEnd(endCondition -> {
            for (Command command : this.children) {
                if (!Boolean.TRUE.equals(finished.get(command))) {
                    command.end(endCondition);
                }
            }
        });

        setStart(() -> {
            for (Command command : this.children) {
                finished.put(command, false);
                command.start();
            }
        });

        setDone(() -> {
            for (Command command : this.children) {
                if (!Boolean.TRUE.equals(finished.get(command))) {
                    return false;
                }
            }
            return true;
        });
    }
}
