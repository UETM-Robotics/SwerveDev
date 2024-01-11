package frc.robot.Util;


/**
 * 2-D Vector Code Representation
 */
public class Vec2d {

    private final double x;
    private final double y;

    public Vec2d(double X, double Y)  {

        this.x = X;
        this.y = Y;

    }

    /**
     * Adds two vectors together
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Vector Sum as Vec2d object
     */
    public static Vec2d add(Vec2d vector1, Vec2d vector2) {

        return new Vec2d( vector1.X() + vector2.X() , vector1.Y() + vector2.Y() ) ;

    }

    /**
     * Subtracts Two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Vector Difference (Vector 1 - Vector 2)
     */
    public static Vec2d subtract(Vec2d vector1, Vec2d vector2) {

        return new Vec2d( vector1.X() - vector2.X() , vector1.Y() - vector2.Y() ) ;

    }

    /**
     * Calculates the Dot Product of two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Dot Product of Vectors
     */
    public static double Dot(Vec2d vector1, Vec2d vector2) {

        return ( vector1.X() * vector2.X() ) + ( vector1.Y() * vector2.Y() ) ;

    }

    /**
     * Calculates the Cross Product of two Vectors
     * @param vector1 First Vector
     * @param vector2 Second Vector
     * @return Cross Product of Vectors as Vec3d object (Vector 1 x Vector 2)
     */
    public static Vec3d Cross(Vec2d vector1, Vec2d vector2) {

        return new Vec3d(0, 0, ( vector1.X() * vector2.Y() ) - ( vector2.X() * vector1.Y() )) ;

    }

    public double X() {
        return this.x ;
    }

    public double Y() {
        return this.y ;
    }

    /**
     * Function to Calculate Length of Vector
     * @return Length of Vector according to Distance Formula as Double
     */
    public double len() {
        return Math.sqrt( (this.x*this.x) + (this.y*this.y) ) ;
    }
    
}
