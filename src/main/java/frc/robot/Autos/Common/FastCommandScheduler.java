package frc.robot.Autos.Common;

import java.util.ArrayList;

public class FastCommandScheduler {

    static ArrayList<FastCommand> fastCommands = new ArrayList<>();

    public static void ScheduleCommand(FastCommand command){
        fastCommands.add(command);
    }

    public void runCurrentCommand(){
        while(!fastCommands.isEmpty()){
            FastCommand currentCommand = fastCommands.get(0);
            currentCommand.init();
            currentCommand.run();
            if(fastCommands.get(0).isFinished()){
                currentCommand.end();
                fastCommands.remove(0);
            }
        }
    }
}
