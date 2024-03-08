package frc.robot.Common;

import java.util.List;

public class NewPath {
    public enum WayPointBehavior{Standard, Velocity};
    public List<Line> lines;
    public WayPointBehavior way = WayPointBehavior.Standard;

    double totaldistance = 0;

    public NewPath(List<Line> lines, WayPointBehavior way){
        this.lines = lines;
        this.way = way;
    }

    public double getTotalDistance(){
        totaldistance = 0;
        for(Line line : lines){
            totaldistance += line.getDistance();
        }
        return totaldistance;
    }
}
