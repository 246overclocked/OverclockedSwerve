package org.usfirst.frc.team246.robot.OverclockedLibraries;

import org.junit.*;
import static org.junit.Assert.*;
import java.util.Arrays;

public class TwoDArrayQuickSorterTest {

	@Test
	public void testQuickSortGeneral() {
		double[][] array = {{1,4},{3,10},{7,3},{5,6},{6,5}};
		double[][] array2 = new double[array.length][];
		for (int i = 0; i < array.length; i++) {  // duplicate array into array2
			array2[i] = array[i];
		}
		TwoDArrayQuickSorter.quickSort(array, 0); // array sorted by index 0
		TwoDArrayQuickSorter.quickSort(array2, 1); // the same array is being sorted here, but by index 1
		for (int i = 0; i < array.length-1; i++) {
			assertTrue(array[i][0] <= array[i+1][0]);
		}
		for (int i = 0; i < array.length-1; i++) {
			assertTrue(array2[i][1] <= array2[i+1][1]);
		}
	}
	
	@Test
	public void testQuickSortRagged() {
		double[][] array = {{1,4,20,15,17,30},{3,10,39,25,28},{7,3,35,11,38,24,22},{5,6,9,1},{6,5,29,31,18}};
		TwoDArrayQuickSorter.quickSort(array, 3);  // index 3 is always available
		for (int i = 0; i < array.length-1; i++) { 
			assertTrue(array[i][3] <= array[i+1][3]);
		}
	}
	
	@Test
	public void testQuickSortConsistency() {
		double[][] array = {{10,10},{11,11},{13,13},{2,2},{1,1},{15,15},{25,25},{100,100}};
		double[][] array2 = new double[array.length][2];
		for (int i = 0; i < array.length; i++) { // duplicate array into array2 for sorting
			array2[i] = array[i];
		}
		TwoDArrayQuickSorter.quickSort(array, 0);
		TwoDArrayQuickSorter.quickSort(array2, 1);
		assertTrue(Arrays.deepEquals(array, array2));
	}
	
	@Test (expected = NullPointerException.class)
	public void testQuickSortNullArrayHandling() {
		double[][] array = null;
		TwoDArrayQuickSorter.quickSort(array, 0);
	}
	
	@Test
	public void testQuickSortShortArrays() {
		double[][] array = {};
		double[][] arrayCopy = {};
		double[][] array2 = {{0,0}};
		double[][] array2Copy = {{0,0}};
		TwoDArrayQuickSorter.quickSort(arrayCopy, 0);
		TwoDArrayQuickSorter.quickSort(array2Copy, 0);
		assertTrue(Arrays.deepEquals(array, arrayCopy));
		assertTrue(Arrays.deepEquals(array2, array2Copy));
	}
	
	@Test (expected = IllegalArgumentException.class)
	public void testQuickSortInvalidIndex() {
		double[][] array = {{0,0},{1,1,1},{2,2},{3,3},{4,4},{5,5}};
		TwoDArrayQuickSorter.quickSort(array, 2); // index 2 is only available in array[1]
	}

}
