package org.usfirst.frc.team246.robot;

import org.junit.Assert;
import org.junit.Test;

public class Vector2DTest {

	@Test
	public void testEqualCartesian() {
		Vector2D cartVec1 = new Vector2D(true, 2, -3);
		Vector2D cartVec2 = new Vector2D(true, 2, -3);
		Vector2D diffCartVecX = new Vector2D(true, 2.1, -3);
		Vector2D diffCartVecY = new Vector2D(true, 2, -3.1);
		
		Assert.assertTrue(Vector2D.equal(cartVec1, cartVec2));
		Assert.assertFalse(Vector2D.equal(cartVec1, diffCartVecX));
		Assert.assertFalse(Vector2D.equal(cartVec1, diffCartVecY));
	}

	@Test
	public void testEqualPolar() {
		Vector2D polVec1 = new Vector2D(false, 2, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 30);
		Vector2D diffPolVecRadius = new Vector2D(false, 2.1, 30);
		Vector2D diffPolVecAngle = new Vector2D(false, 2, 31);
		
		Assert.assertTrue(Vector2D.equal(polVec1, polVec2));
		Assert.assertFalse(Vector2D.equal(polVec1, diffPolVecRadius));
		Assert.assertFalse(Vector2D.equal(polVec1, diffPolVecAngle));
	}
	
	@Test
	public void testEqualPolarEquivalent() {
		Vector2D polVec = new Vector2D(false, 2, 30);
		Vector2D polVecEquivalent = new Vector2D(false, 2, 390);
		Vector2D polVecEquivalent2 = new Vector2D(false, 2, 750);
		Vector2D polVecEquivalent3 = new Vector2D(false, 2, -330);
		
		Assert.assertTrue(Vector2D.equal(polVec, polVecEquivalent));
		Assert.assertTrue(Vector2D.equal(polVec, polVecEquivalent2));
		Assert.assertTrue(Vector2D.equal(polVec, polVecEquivalent3));
	}
	
	@Test
	public void testEqualClose() {
		Vector2D vector = new Vector2D(true, -2.914213562373094, 4.012289773726411);
		Vector2D closeVec = new Vector2D(true, -2.9142, 4.0123);
		Vector2D notCloseVec = new Vector2D(true, -2.91, 4.01);
		Vector2D notCloseVecX = new Vector2D(true, -2.91, 4.0123);
		Vector2D notCloseVecY = new Vector2D(true, -2.9142, 4.01);
		
		Assert.assertTrue(Vector2D.equal(vector, closeVec));
		Assert.assertFalse(Vector2D.equal(vector, notCloseVec));
		Assert.assertFalse(Vector2D.equal(vector, notCloseVecX));
		Assert.assertFalse(Vector2D.equal(vector, notCloseVecY));
	}

	@Test
	public void testEqualNull() {
		Vector2D vec = new Vector2D(true, 2, -3);
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		Vector2D nullVec = null;

		Assert.assertFalse(Vector2D.equal(vec, nullVec));
		Assert.assertFalse(Vector2D.equal(nullVec, vec));
		Assert.assertTrue(Vector2D.equal(nullVec, nullVec));
		Assert.assertTrue(Vector2D.equal(nullVec, zeroVec));
		Assert.assertTrue(Vector2D.equal(zeroVec, nullVec));
	}

	@Test
	public void testPolarToCartSmallAngle() {
		double[] expectedSmallAngle = {-1, Math.sqrt(3)};
		double[] actualSmallAngle = Vector2D.polarToCart(2, 30);
		
		Assert.assertEquals(expectedSmallAngle[0], actualSmallAngle[0], Vector2D.getTolerance());
		Assert.assertEquals(expectedSmallAngle[1], actualSmallAngle[1], Vector2D.getTolerance());
	}

	@Test
	public void testPolarToCartLargeAngle() {
		double[] expectedLargeAngle = {1, Math.sqrt(3)};
		double[] actualLargeAngleNeg = Vector2D.polarToCart(2, -30);
		double[] actualLargeAnglePos = Vector2D.polarToCart(2, 330);

		Assert.assertEquals(expectedLargeAngle[0], actualLargeAngleNeg[0], Vector2D.getTolerance());
		Assert.assertEquals(expectedLargeAngle[1], actualLargeAngleNeg[1], Vector2D.getTolerance());
		
		Assert.assertEquals(expectedLargeAngle[0], actualLargeAnglePos[0], Vector2D.getTolerance());
		Assert.assertEquals(expectedLargeAngle[1], actualLargeAnglePos[1], Vector2D.getTolerance());
	}
	
	public void testPolarToCartAngleEquivalents() {
		double[] expectedLargeAngle = {1, Math.sqrt(3)};
		double[] actualLargeAngleNegEquiv = Vector2D.polarToCart(2, -390);
		double[] actualLargeAnglePosEquiv = Vector2D.polarToCart(2, 690);
		
		Assert.assertEquals(expectedLargeAngle[0], actualLargeAngleNegEquiv[0], Vector2D.getTolerance());
		Assert.assertEquals(expectedLargeAngle[1], actualLargeAngleNegEquiv[1], Vector2D.getTolerance());
		
		Assert.assertEquals(expectedLargeAngle[0], actualLargeAnglePosEquiv[0], Vector2D.getTolerance());
		Assert.assertEquals(expectedLargeAngle[1], actualLargeAnglePosEquiv[1], Vector2D.getTolerance());
	}
	
	@Test
	public void testCloneVectorCartesian() {
		Vector2D cartVec = new Vector2D(true, 2, 3);
		Vector2D cartVecClone = cartVec.cloneVector();
		
		Assert.assertTrue(Vector2D.equal(cartVec, cartVecClone));
	}

	@Test
	public void testCloneVectorPolar() {
		Vector2D polarVec = new Vector2D(false, 2, 30);
		Vector2D polarVecClone = polarVec.cloneVector();
		
		Assert.assertTrue(Vector2D.equal(polarVec, polarVecClone));
	}

	@Test
	public void testNullHandle() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D nullVec = null;
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.nullHandle(cartVec), cartVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.nullHandle(polVec), polVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.nullHandle(nullVec), zeroVec));
	}
	
//	GETTERS
	@Test
	public void testGetXCartesian() {
		Vector2D vec = new Vector2D(true, 2, 3);
		Assert.assertEquals(2, vec.getX(), Vector2D.getTolerance());
	}

	@Test
	public void testGetXPolar() {
		Vector2D vec = new Vector2D(false, 3, 30);
		Assert.assertEquals(-1.5, vec.getX(), Vector2D.getTolerance());
	}

	@Test
	public void testGetYCartesian() {
		Vector2D vec = new Vector2D(true, 2, 3);
		Assert.assertEquals(3, vec.getY(), Vector2D.getTolerance());
	}

	@Test
	public void testGetYPolar() {
		Vector2D vec = new Vector2D(false, 3, 30);
		Assert.assertEquals(2.5981, vec.getY(), Vector2D.getTolerance());
	}

	@Test
	public void testGetAngleCartesian() {
		Vector2D cartVec = new Vector2D(true, -1, Math.sqrt(3));
		Assert.assertEquals(30, cartVec.getAngle(), Vector2D.getTolerance());
	}

	@Test
	public void testGetAnglePolar() {
		Vector2D polarVec = new Vector2D(false, 2, 30);
		Assert.assertEquals(30, polarVec.getAngle(), Vector2D.getTolerance());
	}
	
	@Test
	public void testGetAnglePolarLarge() {
		Vector2D polarVecNeg = new Vector2D(false, 2, 315);
		Vector2D polarVecNeg2 = new Vector2D(false, 2, 675);
		
		Assert.assertEquals(-45, polarVecNeg.getAngle(), Vector2D.getTolerance());
		Assert.assertEquals(-45, polarVecNeg2.getAngle(), Vector2D.getTolerance());
		
		Vector2D polarVecPos = new Vector2D(false, 2, 405);
		Vector2D polarVecPos2 = new Vector2D(false, 2, 765);
		
		Assert.assertEquals(45, polarVecPos.getAngle(), Vector2D.getTolerance());
		Assert.assertEquals(45, polarVecPos2.getAngle(), Vector2D.getTolerance());
	}
	
	@Test
	public void testGetAngleZero() {
		Vector2D cartVec = new Vector2D(true, 0, 0);
		Assert.assertEquals(0, cartVec.getAngle(), Vector2D.getTolerance());
		
		Vector2D polarVec = new Vector2D(false, 0, 123);
		Assert.assertEquals(0, polarVec.getAngle(), Vector2D.getTolerance());
	}

	@Test
	public void testGetMagnitudeCartesian() {
		Vector2D zero = new Vector2D(true, 0, 0);
		Vector2D cartVec = new Vector2D(true, -1, Math.sqrt(3));
		
		Assert.assertEquals(0, zero.getMagnitude(), 0);
		Assert.assertEquals(2, cartVec.getMagnitude(), Vector2D.getTolerance());
	}
	
	@Test
	public void testGetMagnitudePolar() {
		Vector2D polarVec = new Vector2D(false, 2, 30);		
		Assert.assertEquals(2, polarVec.getMagnitude(), Vector2D.getTolerance());
	}

//	SETTERS
	@Test
	public void testSetXCartesian() {
		Vector2D cartVec = new Vector2D(true, 2, 3);
		cartVec.setX(5);
		Assert.assertEquals(5, cartVec.getX(), Vector2D.getTolerance());
	}

	@Test
	public void testSetXPolar() {
		Vector2D polVec = new Vector2D(false, 2, 30);
		polVec.setX(5);
		Assert.assertEquals(5, polVec.getX(), Vector2D.getTolerance());
	}

	@Test
	public void testSetYCartesian() {
		Vector2D cartVec = new Vector2D(true, 2, 3);
		cartVec.setY(5);
		Assert.assertEquals(5, cartVec.getY(), Vector2D.getTolerance());
	}

	@Test
	public void testSetYPolar() {
		Vector2D polVec = new Vector2D(false, 2, 30);
		polVec.setY(5);
		Assert.assertEquals(5, polVec.getY(), Vector2D.getTolerance());
	}

	@Test
	public void testSetAngleCartesian() {
		Vector2D cartVec = new Vector2D(true, -2, -3);
		cartVec.setAngle(210);
		Assert.assertEquals(-150, cartVec.getAngle(), Vector2D.getTolerance());
	}
	
	@Test
	public void testSetAnglePolar() {
		Vector2D polVec = new Vector2D(false, 2, 30);
		polVec.setAngle(-30);
		Assert.assertEquals(-30, polVec.getAngle(), Vector2D.getTolerance());
	}

	@Test
	public void testSetMagnitudeCartesian() {
		Vector2D cartVec = new Vector2D(true, -2, -3);
		cartVec.setMagnitude(2);
		Assert.assertEquals(2, cartVec.getMagnitude(), Vector2D.getTolerance());
	}

	@Test
	public void testSetMagnitudePolar() {
		Vector2D polVec = new Vector2D(false, 2, 30);
		polVec.setMagnitude(2);
		Assert.assertEquals(2, polVec.getMagnitude(), Vector2D.getTolerance());
	}

//	MATH OPERATIONS
	@Test
	public void testAddVectorsCartesian() {
		Vector2D cartVec1 = new Vector2D(true, 3, 5);
		Vector2D cartVec2 = new Vector2D(true, -5, 3);
		Vector2D expectedCartSum = new Vector2D(true, -2, 8);

		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(cartVec1, cartVec2), expectedCartSum));
	}

	@Test
	public void testAddVectorsPolar() {
		Vector2D polVec1 = new Vector2D(false, 3, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 45);
		Vector2D expectedPolSum = new Vector2D(true, -2.9142, 4.0123);

		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(polVec1, polVec2), expectedPolSum));
	}
	
	@Test
	public void testAddVectorsBoth() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D expectedSum = new Vector2D(true, 1.5858, 6.4142);

		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(polVec, cartVec), expectedSum));
	}
	
	@Test
	public void testAddVectorsNull() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D nullVec = null;

		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(nullVec, cartVec), cartVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(cartVec, nullVec), cartVec));
		
		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(nullVec, polVec), polVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.addVectors(polVec, nullVec), polVec));
	}
	
	@Test
	public void testSubtractVectorsCartesian() {
		Vector2D cartVec1 = new Vector2D(true, 3, 5);
		Vector2D cartVec2 = new Vector2D(true, -5, 3);
		Vector2D expectedCartDiff = new Vector2D(true, 8, 2);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(cartVec1, cartVec2), expectedCartDiff));
	}

	@Test
	public void testSubtractVectorsPolar() {
		Vector2D polVec1 = new Vector2D(false, 3, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 45);
		Vector2D expectedPolDiff = new Vector2D(true, -0.085786, 1.183863);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(polVec1, polVec2), expectedPolDiff));
	}
	
	@Test
	public void testSubtractVectorsBoth() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D expectedDiff = new Vector2D(true, -4.4142, -3.5858);

		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(polVec, cartVec), expectedDiff));
	}
	
	@Test
	public void testSubtractVectorsNull() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D negPolVec = new Vector2D(false, 2, 225);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D negCartVec = new Vector2D(true, -3, -5);
		Vector2D nullVec = null;

		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(cartVec, nullVec), cartVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(nullVec, cartVec), negCartVec));
		
		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(polVec, nullVec), polVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.subtractVectors(nullVec, polVec), negPolVec));
	}
	
	@Test
	public void testUnitVectorCartesian() {
		Vector2D cartOriginal = new Vector2D(true, 3, 4);
		Vector2D expectedCartUnit = new Vector2D(true, 3.0/5, 4.0/5);
		
		Assert.assertTrue(Vector2D.equal(cartOriginal.unitVector(), expectedCartUnit));
	}

	@Test
	public void testUnitVectorPolar() {
		Vector2D polOriginal = new Vector2D(false, 3, 30);
		Vector2D expectedPolUnit = new Vector2D(true, -0.50000, 0.86603);
		
		Assert.assertTrue(Vector2D.equal(polOriginal.unitVector(), expectedPolUnit));
	}
	
	@Test
	public void testUnitVectorZero() {
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		Assert.assertTrue(Vector2D.equal(zeroVec.unitVector(), zeroVec));
	}

	@Test
	public void testDotProductCartesian() {
		Vector2D cartVec1 = new Vector2D(true, 3, 7);
		Vector2D cartVec2 = new Vector2D(true, -5, 3);
		double expectedCart = 6;

		Assert.assertEquals(Vector2D.dotProduct(cartVec1, cartVec2), expectedCart, Vector2D.getTolerance());
	}
	
	@Test
	public void testDotProductPolar() {
		Vector2D polVec1 = new Vector2D(false, 3, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 45);
		double expectedPol = 5.7956;
		
		Assert.assertEquals(Vector2D.dotProduct(polVec1, polVec2), expectedPol, Vector2D.getTolerance());
	}

	@Test
	public void testDotProductBoth() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		double expected = 2.8284;

		Assert.assertEquals(Vector2D.dotProduct(polVec, cartVec), expected, Vector2D.getTolerance());
	}
	
	@Test
	public void testDotProductNull() {
		Vector2D polVec = new Vector2D(false, 2, 45);
		Vector2D cartVec = new Vector2D(true, 3, 5);
		Vector2D nullVec = null;

		Assert.assertEquals(Vector2D.dotProduct(nullVec, cartVec), 0, 0);
		Assert.assertEquals(Vector2D.dotProduct(cartVec, nullVec), 0, 0);
		
		Assert.assertEquals(Vector2D.dotProduct(nullVec, polVec), 0, 0);
		Assert.assertEquals(Vector2D.dotProduct(polVec, nullVec), 0, 0);
	}
	
	@Test
	public void testParallelProjectionCartesianSimple() {
		Vector2D cartVec = new Vector2D(true, 3, 4);
		Vector2D cartXComponent = new Vector2D(true, 3, 0);
		Vector2D cartYComponent = new Vector2D(true, 0, 4);
		
		Vector2D xAxis = new Vector2D(true, 2, 0); // non-unit vector
		Vector2D yAxis = new Vector2D(true, 0, 2); // non-unit vector
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(cartVec, xAxis), cartXComponent));
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(cartVec, yAxis), cartYComponent));
	}
	
	@Test
	public void testParallelProjectionCartesian() {
		Vector2D vector1 = new Vector2D(true, 3, 7);
		Vector2D vector2 = new Vector2D(true, -5, 3);
		Vector2D expected = new Vector2D(true, -15.0/17, 9.0/17);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(vector1, vector2), expected));
	}
	
	@Test
	public void testParallelProjectionPolarSimple() {
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D polXComponent = new Vector2D(true, -1.5, 0);
		Vector2D polYComponent = new Vector2D(true, 0, 2.5981);
		
		Vector2D xAxis = new Vector2D(true, 2, 0); // non-unit vector
		Vector2D yAxis = new Vector2D(true, 0, 2); // non-unit vector
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(polVec, xAxis), polXComponent));
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(polVec, yAxis), polYComponent));
	}

	@Test
	public void testParallelProjectionPolar() {
		Vector2D polVec1 = new Vector2D(false, 3, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 45);
		Vector2D expected = new Vector2D(true, -2.04905, 2.04905);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(polVec1, polVec2), expected));
	}
	
	@Test
	public void testParallelProjectionBoth() {
		Vector2D cartVec = new Vector2D(true, 3, 7);
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D expected = new Vector2D(true, -2.28109, 3.95099);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(cartVec, polVec), expected));
	}
	
	@Test
	public void testParallelProjectionNull() {
		Vector2D cartVec = new Vector2D(true, 3, 7);
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D nullVec = null;
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(nullVec, cartVec), zeroVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(cartVec, nullVec), zeroVec));
		
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(nullVec, polVec), zeroVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.parallelProjection(polVec, nullVec), zeroVec));
	}
	
	@Test
	public void testPerpendicularProjectionCartesianSimple() {
		Vector2D cartVec = new Vector2D(true, 3, 4);
		Vector2D cartXComponent = new Vector2D(true, 3, 0);
		Vector2D cartYComponent = new Vector2D(true, 0, 4);
		
		Vector2D xAxis = new Vector2D(true, 2, 0); // non-unit vector
		Vector2D yAxis = new Vector2D(true, 0, 2); // non-unit vector
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(cartVec, xAxis), cartYComponent));
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(cartVec, yAxis), cartXComponent));
	}
	
	@Test
	public void testPerpendicularProjectionCartesian() {
		Vector2D vector1 = new Vector2D(true, 3, 7);
		Vector2D vector2 = new Vector2D(true, -5, 3);
		Vector2D expected = new Vector2D(true, 3.8824, 6.4706);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(vector1, vector2), expected));
	}
	
	@Test
	public void testPerpendicularProjectionPolarSimple() {
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D polXComponent = new Vector2D(true, -1.5, 0);
		Vector2D polYComponent = new Vector2D(true, 0, 2.5981);
		
		Vector2D xAxis = new Vector2D(true, 2, 0); // non-unit vector
		Vector2D yAxis = new Vector2D(true, 0, 2); // non-unit vector
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(polVec, xAxis), polYComponent));
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(polVec, yAxis), polXComponent));
	}
	
	@Test
	public void testPerpendicularProjectionPolar() {
		Vector2D polVec1 = new Vector2D(false, 3, 30);
		Vector2D polVec2 = new Vector2D(false, 2, 45);
		Vector2D expected = new Vector2D(true, 0.54905, 0.54903);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(polVec1, polVec2), expected));
	}
	
	@Test
	public void testPerpendicularProjectionBoth() {
		Vector2D cartVec = new Vector2D(true, 3, 7);
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D expected = new Vector2D(true, 5.2811, 3.0490);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(cartVec, polVec), expected));
	}
	
	
	
	@Test
	public void testPerpendicularProjectionOntoZero() {
		Vector2D vector = new Vector2D(true, 3, 7);
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(vector, zeroVec), zeroVec));
	}
	
	@Test
	public void testPerpendicularProjectionNull() {
		Vector2D cartVec = new Vector2D(true, 3, 7);
		Vector2D polVec = new Vector2D(false, 3, 30);
		Vector2D nullVec = null;
		Vector2D zeroVec = new Vector2D(true, 0, 0);
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(nullVec, cartVec), zeroVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(cartVec, nullVec), zeroVec));
		
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(nullVec, polVec), zeroVec));
		Assert.assertTrue(Vector2D.equal(Vector2D.perpendicularProjection(polVec, nullVec), zeroVec));
	}

}
