package org.usfirst.frc.team246.robot;

/**
 * 
 * @author Jacob Nazarenko
 * - Creds to Gaddis's Java Textbook for the structure of this code
 */
public class TwoDArrayQuickSorter {
	
	/**
     * This QuickSort method sorts a two-dimensional array by a given index of 
     * each of its inner arrays. 
     * 
     * For example: if you had the array {{1,4},{3,10},{7,3},{5,6}}, and sorted 
     * it by index 0, you would end up with {{1,4},{3,10},{5,6},{7,3}}, where
     * the values of the array[i][0] are in ascending order: 1, 3, 5, 7.
     * If you sorted this array by index 1, however, you would end up with
     * {{7,3},{1,4},{5,6},{3,10}}, where the values of the array[i][1] are in 
     * ascending order: 3, 4, 6, 10. 
     * 
     * @param array
     *            The two-dimensional array to sort.
     * @param index
     * 			  The index of inner arrays to sort. For example: if you are 
     * 			  sorting by index 0, the inner arrays will be sorted by their 
     * 			  first values.	
     * @throws
     * 			  NullPointerException
     * 			  IllegalArgumentException 
     */

    public static void quickSort(double array[][], int index) throws NullPointerException, IllegalArgumentException {
    	if (array == null) {
    		throw new NullPointerException();
    	} else if (array.length == 0 || array.length == 1) {
			return;
		} else {
			for (int i = 0; i < array.length; i++) {
				if (index > array[i].length - 1) {
					throw new IllegalArgumentException();
				}
			}
			doQuickSort(array, 0, array.length - 1, index);
		}
    }

    private static void doQuickSort(double array[][], int start, int end, int index) {
        int pivotPoint;

        if (start < end) {
            pivotPoint = partition(array, start, end, index);
            doQuickSort(array, start, pivotPoint - 1, index);
            doQuickSort(array, pivotPoint + 1, end, index);
        }
    }

    private static int partition(double array[][], int start, int end, int index) {
        double pivotValue;
        int endOfLeftList;
        int mid;

        mid = (start + end) / 2;
        swap(array, start, mid);
        pivotValue = array[start][index];
        endOfLeftList = start;
        for (int scan = start + 1; scan <= end; scan++) {
            if (array[scan][index] < pivotValue) {
                endOfLeftList++;
                swap(array, endOfLeftList, scan);
            }
        }
        swap(array, start, endOfLeftList);
        return endOfLeftList;
    }

    private static void swap(double[][] a, int m, int n) {
        double[] tmp = a[m];
        a[m] = a[n];
        a[n] = tmp;
    }
}