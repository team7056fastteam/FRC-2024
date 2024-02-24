package frc.robot.Autos.Common;

public class NewPathRunner {
    int length = 0;
    int currentLine = 0;
    public NewPathRunner(NewPath path){
        length = path.lines.size();
    }

    public void runPath(){
        if(length == currentLine){
            //endpoint
        }
        else{
            //waypoint
        }
    }
}
