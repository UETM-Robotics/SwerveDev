package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 3-D Vector Code Representation
 */
public class Vec3d {

    private final double x;
    private final double y;
    private final double z;

    public Vec3d()  {

        this(0,0,0);

    }

    public Vec3d(double X, double Y, double Z)  {

        this.x = X;
        this.y = Y;
        this.z = Z;

    }

    /**
     * Adds two vectors together
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Vector Sum as Vec3d object
     */
    public static Vec3d add(Vec3d vector1, Vec3d vector2) {

        return new Vec3d( vector1.X() + vector2.X(), 
                        vector1.Y() + vector2.Y(),
                        vector1.Z() + vector2.Z() ) ;

    }

    /**
     * Subtracts Two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Vector Difference as Vec3d object (Vector 1 - Vector 2)
     */
    public static Vec3d subtract(Vec3d vector1, Vec3d vector2) {

        return new Vec3d( vector1.X() - vector2.X(), 
                        vector1.Y() - vector2.Y(),
                        vector1.Z() - vector2.Z() ) ;

    }

    /**
     * Calculates the Dot Product of two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Dot Product of Vectors
     */
    public static double Dot(Vec3d vector1, Vec3d vector2) {

        return ( vector1.X() * vector2.X() ) + 
                ( vector1.Y() * vector2.Y() ) + 
                ( vector1.Z() * vector2.Z() ) ;

    }

    /**
     * Calculates the Cross Product of two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Cross Product of Vectors as Vec3d object (Vector 1 x Vector 2)
     */
    public static Vec3d Cross(Vec3d vector1, Vec3d vector2) {

        return new Vec3d(( vector1.Y() * vector2.Z() ) - ( vector1.Z() * vector2.Y() ),
                        ( vector1.X() * vector2.Z() ) - ( vector1.Z() * vector2.X() ),
                        ( vector1.X() * vector2.Y() ) - ( vector1.Y() * vector2.X() )) ;

    }

    public double X() {
        return this.x ;
    }

    public double Y() {
        return this.y ;
    }

    public double Z() {
        return this.z ;
    }

    public Rotation2d Theta() {
        return Rotation2d.fromDegrees(Z());
    }

    public Pose2d asPose2d() {
        return new Pose2d(this.x, this.y, this.Theta());
    }

    public static Vec3d fromPose2d(Pose2d pose) {
        return new Vec3d(pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getDegrees());
    }

    /**
     * Function to Return the X and Y component of 3-D Vector
     * as a Vec2d Object
     * @return X and Y components as Vec2d Object
     */
    public Vec2d X_Y_Coord() {
        return new Vec2d(this.x, this.y) ;
    }

    /**
     * Function to Calculate Length of Vector
     * @return Length of Vector according to Distance Formula as Double
     */
    public double len() {
        return Math.sqrt( (this.x*this.x) + (this.y*this.y) + (this.z*this.z) ) ;
    }
    
}
