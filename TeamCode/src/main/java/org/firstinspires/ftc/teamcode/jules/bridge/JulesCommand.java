// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesCommand.java
package org.firstinspires.ftc.teamcode.jules.bridge;

public class JulesCommand {
    /**
     * This enum defines all the "virtual OpModes" or actions our tuner can perform.
     */
    public enum Command {
        IDLE,
        MANUAL_DRIVE_TEST,
        FEEDFORWARD_TEST
        // We will add more commands here as we build more tuning steps
    }

    private static volatile Command currentCommand = Command.IDLE;
    private static final String[] commandNames;

    // A static block to create a clean list of names from the enum
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

    public static void setCommand(Command command) {
        currentCommand = command;
    }

    public static void setCommand(String commandName) {
        try {
            currentCommand = Command.valueOf(commandName);
        } catch (IllegalArgumentException e) {
            System.out.println("JULES: Warning - received invalid command name: " + commandName);
            currentCommand = Command.IDLE;
        }
    }

    public static Command getCommand() {
        return currentCommand;
    }
}