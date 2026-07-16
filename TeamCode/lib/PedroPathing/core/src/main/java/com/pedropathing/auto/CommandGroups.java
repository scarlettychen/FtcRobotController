package com.pedropathing.auto;

/**
 * Sequential and parallel composition — one file for both group types.
 */
final class CommandGroups {
    private CommandGroups() {}

    static final class Sequential extends BaseAutoCommand {
        private final AutoCommand[] children;
        private int index;

        Sequential(AutoCommand... children) {
            this.children = children == null ? new AutoCommand[0] : children;
        }

        @Override
        public void initialize() {
            index = 0;
            if (children.length > 0) children[0].initialize();
        }

        @Override
        public void execute() {
            if (index >= children.length) return;
            AutoCommand current = children[index];
            current.execute();
            if (current.isFinished()) {
                current.end();
                index++;
                if (index < children.length) children[index].initialize();
            }
        }

        @Override
        public void end() {
            if (index < children.length) children[index].end();
        }

        @Override
        public boolean isFinished() {
            return index >= children.length;
        }
    }

    static final class Parallel extends BaseAutoCommand {
        private final AutoCommand[] children;
        private final boolean[] finished;

        Parallel(AutoCommand... children) {
            this.children = children == null ? new AutoCommand[0] : children;
            this.finished = new boolean[this.children.length];
        }

        @Override
        public void initialize() {
            for (int i = 0; i < children.length; i++) {
                finished[i] = false;
                children[i].initialize();
            }
        }

        @Override
        public void execute() {
            for (int i = 0; i < children.length; i++) {
                if (finished[i]) continue;
                children[i].execute();
                if (children[i].isFinished()) {
                    children[i].end();
                    finished[i] = true;
                }
            }
        }

        @Override
        public void end() {
            for (int i = 0; i < children.length; i++) {
                if (!finished[i]) children[i].end();
            }
        }

        @Override
        public boolean isFinished() {
            for (boolean f : finished) {
                if (!f) return false;
            }
            return true;
        }
    }
}
