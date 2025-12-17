package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class GamePiece {

    public Translation2d position;
    public Translation2d velocity;

    public final double radius;
    public final double mass;

    public GamePiece(
        Translation2d startPosition,
        double radius,
        double mass
    ) {
        this.position = startPosition;
        this.velocity = new Translation2d();
        this.radius = radius;
        this.mass = mass;
    }
}
