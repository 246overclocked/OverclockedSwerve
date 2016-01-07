package org.usfirst.frc.team246.robot;
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


/**
 *
 * @author michaelsilver
 */
public class Vector2D {
    
    private double x;
    private double y;
    private static final double TOLERANCE = 0.001;
    private boolean cartesian;

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
    
    @Override
	public String toString() {
		if (cartesian) {
			return String.format("Vector2D [x=%.3f, y=%.3f]", x, y); // rounds to third decimal
		} else {
			return String.format("Vector2D [r=%.3f, theta=%.3f]", getMagnitude(), getAngle()); // rounds to third decimal
		}
	}

	public static double[] polarToCart(double r, double theta){
    	theta += 90;
        double[] cartCoords = {r*Math.cos(Math.toRadians(theta)), r*Math.sin(Math.toRadians(theta))};
        return cartCoords;
    }
    
    public Vector2D cloneVector(){
    	return new Vector2D(true, x, y);
    }
    
//    GETTERS
    public double getX(){
        return x;
    } 
    
    public double getY(){
        return y;
    }
    
    public static double getTolerance() {
		return TOLERANCE;
	}

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
    
    public double getMagnitude() {
        return Math.sqrt(x*x + y*y);
    }
    
//    SETTERS
    public void setX(double x){
        this.x = x;
    }
    
    public void setY(double y){
        this.y = y;
    }
    
    public void setAngle(double angle)
    {
        double[] newCoords = Vector2D.polarToCart(getMagnitude(), angle);
        x = newCoords[0];
        y = newCoords[1];
    }
    
    public void setMagnitude(double magnitude)
    {
        double[] newCoords = Vector2D.polarToCart(magnitude, getAngle());
        x = newCoords[0];
        y = newCoords[1];
    }
       
//    MATH OPERATIONS
    public static boolean equal(Vector2D vector1, Vector2D vector2) {
    	return (Math.abs(vector1.x - vector2.x) < TOLERANCE && Math.abs(vector1.y - vector2.y) < TOLERANCE);
    }
    
    public static Vector2D addVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() + vector2.getX(), vector1.getY() + vector2.getY());
        return sum;
    }
    
    public static Vector2D subtractVectors(Vector2D vector1, Vector2D vector2){
        Vector2D sum = new Vector2D(true, vector1.getX() - vector2.getX(), vector1.getY() - vector2.getY());
        return sum;
    }
    
    public Vector2D unitVector(){
        Vector2D unitVector = new Vector2D(true, x/getMagnitude(), y/getMagnitude());
        return unitVector;
    }
    
    public static double dotProduct(Vector2D vector1, Vector2D vector2){
    	return vector1.getX()*vector2.getX() + vector1.getY()*vector2.getY();
    }
    
//    project vector1 onto vector2
    public static Vector2D parallelProjection(Vector2D vector1, Vector2D vector2){
//    	using the geometric definition of the dot product, see
//    	https://en.wikipedia.org/wiki/Dot_product#Scalar_projection_and_first_properties
    	Vector2D vec2Unit = vector2.unitVector();
    	double magnitudeOfProjection = dotProduct(vector1, vec2Unit);
    	Vector2D projection = vec2Unit.cloneVector();
    	projection.setMagnitude(magnitudeOfProjection);
    	return projection;
    }
    
//    the other (than the projection) component of vector1 with a coordinate system relative to vector2
    public static Vector2D perpendicularProjection(Vector2D vector1, Vector2D vector2){
    	return subtractVectors(vector1, parallelProjection(vector1, vector2));
    }
}
