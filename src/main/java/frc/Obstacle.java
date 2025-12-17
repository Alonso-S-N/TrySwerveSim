package frc;

public class Obstacle {

    public final double minX;
    public final double minY;
    public final double maxX;
    public final double maxY;

    public Obstacle(double x, double y, double width, double height) {
        this.minX = x;
        this.minY = y;
        this.maxX = x + width;
        this.maxY = y + height;
    }
}
