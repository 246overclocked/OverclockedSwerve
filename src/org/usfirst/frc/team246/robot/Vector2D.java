package org.usfirst.frc.team246.robot;

/**
 *
 * @author michaelsilver
 */
public class Vector2D {
    
    private double x;
    private double y;
    private static final double TOLERANCE = 0.001;
    private boolean cartesian;

	/**
	 * Creates a new {@link Vector2D} object. Allows both polar and Cartesian as
	 * input coordinates.
	 * 
	 * @param cartesian
	 *            If true, then coordinates are interpreted as Cartesian. If
	 *            false, coordinates interpreted as polar.
	 * @param abscissa
	 *            If Cartesian, then x. If polar, then radius.
	 * @param ordinate
	 *            If Cartesian, then y. If polar, then angle.
	 */
    public Vector2D(boolean cartesian, double abscissa, double ordinate){
        this.cartesian = cartesian;

        if(cartesian){
            x = abscissa;
            y = ordinate;
        } else {
            double[] coords = polarToCart(abscissa, ordinate);
            x = coords[0];
            y = coords[1];
        }
    }
    
    /* (non-Javadoc)
     * @see java.lang.Object#toString()
     */
    @Override
	public String toString() {
		if (cartesian) {
			return String.format("Vector2D [x=%.3f, y=%.3f]", x, y); // rounds to third decimal
		} else {
			return String.format("Vector2D [r=%.3f, theta=%.3f]", getMagnitude(), getAngle()); // rounds to third decimal
		}
	}

	/**
	 * Converts polar coordinates into Cartesian coordinates.
	 * 
	 * @param r
	 *            Radius
	 * @param theta
	 *            Angle in degrees
	 * @return An array of the form [x,y] with the equivalent Cartesian
	 *         coordinates.
	 */
	public static double[] polarToCart(double r, double theta){
    	theta += 90;
        return new double[] {r*Math.cos(Math.toRadians(theta)), r*Math.sin(Math.toRadians(theta))};
    }
    
	/**
	 * Copies a {@link Vector2D} into a new one.
	 *
	 * @return A new copy of the vector
	 * 
	 * @throws NullPointerException
	 */
    public Vector2D cloneVector(){
    	return new Vector2D(true, x, y);
    }
    
	/**
	 * Handles null vectors by converting them to zero vectors.
	 * 
	 * If a non-null vector is given, returns original vector.
	 * 
	 * @param vector
	 *            Vector to null handle
	 * @return A non-null vector
	 */
    public static Vector2D nullHandle(Vector2D vector) {
		if (vector == null) {
			return new Vector2D(true, 0, 0);
		} else {
			return vector;
		}
	}
    
//    GETTERS
	/**
	 * Gets the x-coordinate in Cartesian coordinates.
	 * 
	 * @return The x-coordinate
	 */
    public double getX(){
        return x;
    } 
    
	/**
	 * Gets the y-coordinate in Cartesian coordinates.
	 * 
	 * @return The y-coordinate
	 */
    public double getY(){
        return y;
    }
    
	/**
	 * Gets the default tolerance for Vector2D.equalVector()
	 * 
	 * @return Default tolerance
	 */
    public static double getTolerance() {
		return TOLERANCE;
	}

	/**
	 * Gets the angle in polar coordinates.
	 * 
	 * @return Angle in degrees
	 */
    public double getAngle(){
    	if (x==0 && y==0) {
    		return 0;
    	} else {
	        double angle = Math.toDegrees(Math.atan2(y, x)) - 90;
	        if(angle < -180) return angle + 360;
	        else if(angle > 180) return angle - 360;
	        else return angle;
    	}
    }
    
	/**
	 * Gets the magnitude of the vector.
	 * 
	 * @return Magnitude
	 */
    public double getMagnitude() {
        return Math.sqrt(x*x + y*y);
    }
    
//    SETTERS
	/**
	 * Sets the x-coordinate of the vector (in Cartesian coordinates).
	 * 
	 * @param x
	 *            The new x-coordinate
	 */
    public void setX(double x){
        this.x = x;
    }
    
	/**
	 * Sets the y-coordinate of the vector (in Cartesian coordinates).
	 * 
	 * @param y
	 *            The new y-coordinate
	 */
    public void setY(double y){
        this.y = y;
    }
    
	/**
	 * Sets the angle of the vector (in Polar coordinates).
	 * 
	 * @param angle
	 *            The new angle in degrees
	 */
    public void setAngle(double angle)
    {
        double[] newCoords = Vector2D.polarToCart(getMagnitude(), angle);
        x = newCoords[0];
        y = newCoords[1];
    }
    
	/**
	 * Sets the magnitude of the vector.
	 * 
	 * @param magnitude
	 *            The new magnitude
	 */
    public void setMagnitude(double magnitude)
    {
        double[] newCoords = Vector2D.polarToCart(magnitude, getAngle());
        x = newCoords[0];
        y = newCoords[1];
    }
       
//    MATH OPERATIONS
	/**
	 * Tests whether two vectors are equivalent.
	 * 
	 * Null vectors are treated as zero vectors.
	 * 
	 * @param vector1
	 *            First vector
	 * @param vector2
	 *            Second vector
	 * @return True if vectors are equivalent, false otherwise.
	 */
    public static boolean equal(Vector2D vector1, Vector2D vector2) {
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
    	return (Math.abs(vector1.x - vector2.x) < TOLERANCE && Math.abs(vector1.y - vector2.y) < TOLERANCE);
    }
    
	/**
	 * Adds two vectors together and returns the result.
	 * 
	 * Null vectors are treated as zero vectors.
	 * 
	 * @param vector1
	 *            The first vector in the sum
	 * @param vector2
	 *            The second vector in the sum
	 * @return The sum of the two vectors
	 */
    public static Vector2D addVectors(Vector2D vector1, Vector2D vector2){
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
        Vector2D sum = new Vector2D(true, vector1.getX() + vector2.getX(), vector1.getY() + vector2.getY());
        return sum;
    }
    
	/**
	 * Subtracts two vectors and returns the result.
	 * 
	 * Null vectors are treated as zero vectors.
	 * 
	 * @param vector1
	 *            The first vector in the difference
	 * @param vector2
	 *            The second vector in the difference
	 * @return The difference of the two vectors
	 */
    public static Vector2D subtractVectors(Vector2D vector1, Vector2D vector2){
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
    	Vector2D sum = new Vector2D(true, vector1.getX() - vector2.getX(), vector1.getY() - vector2.getY());
        return sum;
    }
    
	/**
	 * Finds the unit vector of the given vector.
	 * 
	 * @return A unit vector
	 */
    public Vector2D unitVector(){
    	if (getMagnitude() == 0) {
    		return new Vector2D(true, 0, 0); // then this is the zero vector
    	} else {
    		return new Vector2D(true, x/getMagnitude(), y/getMagnitude());
    	}
    }
    
	/**
	 * Finds the dot product of two vectors.
	 * 
	 * Null vectors are treated as zero vectors.
	 * 
	 * @param vector1
	 *            The first vector in the dot product
	 * @param vector2
	 *            The second vector in the dot product
	 * @return The scalar obtained by dotting the two vectors
	 */
    public static double dotProduct(Vector2D vector1, Vector2D vector2){
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
    	return vector1.getX()*vector2.getX() + vector1.getY()*vector2.getY();
    }
    
	/**
	 * Finds the parallel projection of one vector on another.
	 * 
	 * Null vectors are treated as zero vectors.
	 * 
	 * @param vector1
	 *            The vector to project
	 * @param vector2
	 *            The vector onto which to project
	 * @return The projection of the first vector onto the second
	 */
    public static Vector2D parallelProjection(Vector2D vector1, Vector2D vector2){
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
//    	using the geometric definition of the dot product, see
//    	https://en.wikipedia.org/wiki/Dot_product#Scalar_projection_and_first_properties
    	Vector2D vec2Unit = vector2.unitVector();
    	double magnitudeOfProjection = dotProduct(vector1, vec2Unit);
    	Vector2D projection = vec2Unit.cloneVector();
    	projection.setMagnitude(magnitudeOfProjection);
    	return projection;
    }
    
	/**
	 * Finds the perpendicular projection of one vector onto another.
	 * 
	 * Null vectors are treated as zero vectors. A projection onto a zero vector
	 * is always zero.
	 * 
	 * @param vector1
	 *            The vector to project
	 * @param vector2
	 *            The vector onto which to project
	 * @return The component of the first vector that is perpendicular to the
	 *         second vector.
	 */
    public static Vector2D perpendicularProjection(Vector2D vector1, Vector2D vector2){
//      the other (than the projection) component of vector1 with a coordinate system relative to vector2
    	vector1 = nullHandle(vector1);
    	vector2 = nullHandle(vector2);
    	
    	
    	if (vector2.getMagnitude() == 0) {
    		return new Vector2D(true, 0, 0);
    	} else {
    		return subtractVectors(vector1, parallelProjection(vector1, vector2));
    	}
    }
}
