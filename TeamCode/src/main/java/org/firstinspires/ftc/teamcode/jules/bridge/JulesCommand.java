// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesCommand.java
package org.firstinspires.ftc.teamcode.jules.bridge;

public class JulesCommand {
    public enum Command {
        IDLE,
        MANUAL_DRIVE_TEST,
        FEEDFORWARD_TEST
    }

    private static volatile Command currentCommand = Command.IDLE;
    private static final String[] commandNames;

    static {
        Command[] commands = Command.values();
        commandNames = new String[commands.length];
        for (int i = 0; i < commands.length; i++) {
            commandNames[i] = commands[i].name();
        }
    }

    public static String[] getCommandNames() {
        return commandNames;
    }

    public static void setCommand(String commandName) {
        try {
            currentCommand = Command.valueOf(commandName);
        } catch (IllegalArgumentException e) {
            currentCommand = Command.IDLE;
        }
    }

    public static Command getCommand() {
        return currentCommand;
    }
}