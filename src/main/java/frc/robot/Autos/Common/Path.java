package frc.robot.Autos.Common;

public class Path {
    public enum WayPointBehavior{Standard, Velocity};
    public double[][] points;
    public WayPointBehavior way = WayPointBehavior.Standard;
    public Path(double[][] points, WayPointBehavior way){
        this.points = points;
        this.way = way;
    }
}
