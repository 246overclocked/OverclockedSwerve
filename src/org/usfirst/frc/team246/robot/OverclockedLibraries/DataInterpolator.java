package org.usfirst.frc.team246.robot.OverclockedLibraries;

/**
 * This is a generic class for the interpolation of data stored in a 2D array. 
 * The 2D data array is a parameter of this object's constructor. The array used
 * in the constructor MUST BE OF LENGTH 1 OR GREATER!! Otherwise, an
 * IllegalArgumentException will be thrown.
 * 
 * @author Jacob Nazarenko
 *
 */
public class DataInterpolator {
	
	/**
	 * This method can find a value based on piecewise linear functions generated
	 * from the data points entered. 
	 * 
	 * @param value
	 * 			The value whose y-coordinate you would like to return
	 * @return 
	 * 			The value's y-coordinate on the corresponding piecewise linear function
	 * @throws
	 * 			NullPointerException
	 * 			IllegalArgumentException
	 */

    public static double interpolateValue(double value, double[][] dataArray) throws NullPointerException, IllegalArgumentException {
    	double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    	if (dataArray == null) {
			throw new NullPointerException();
		} else if (dataArray.length == 0) {
			throw new IllegalArgumentException();
		} else if (dataArray.length == 1) { // defaults to the only y-coordinate entered if array has only 1 element
			return dataArray[0][1];
		}
    	TwoDArrayQuickSorter.quickSort(dataArray, 0); // sort array by first value in each inner array
    	if (value <= dataArray[0][0]) {  // uses lowest two points if value is below range of array[i][0] values
    		x1 = dataArray[0][0];
    		x2 = dataArray[1][0];
    		y1 = dataArray[0][1];
    		y2 = dataArray[1][1];
    	}
    	else if (value >= dataArray[dataArray.length-1][0]) { // uses highest two points if value is above range of array[i][0] values
    		x1 = dataArray[dataArray.length-2][0];
    		x2 = dataArray[dataArray.length-1][0];
    		y1 = dataArray[dataArray.length-2][1];
    		y2 = dataArray[dataArray.length-1][1];
    	}
    	else {  // otherwise uses a range within the values entered
    		for (int i = 0; i < dataArray.length; i++) { 
        		if (dataArray[i][0] > value) {
        			x1 = dataArray[i-1][0];
            		x2 = dataArray[i][0];
            		y1 = dataArray[i-1][1];
            		y2 = dataArray[i][1];
            		break;
        		}
        	}
    	}
    	double slope = (y2 - y1)/(x2 - x1);
		double intercept = (y2 - (x2*slope));
		return (slope*value) + intercept;
    }
}
