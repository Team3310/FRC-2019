package frc.team3310.utility;

import java.io.PrintWriter;

import frc.team3310.robot.Constants;

public class MotionProfileSpinMove {

    private MotionProfileBoxCar mp;
    private double initialLinearVelocity;
    private double spinAngleScaleFactor;
    private double previousRightVelocity;
    private double previousLeftVelocity;
    private MotionProfilePoint mpPoint = new MotionProfilePoint();

	public MotionProfileSpinMove(double initialLinearVelocity, double startAngle, double endAngle, double maxTurnVelocity, double itp, double t1, double t2) {
        this.initialLinearVelocity = initialLinearVelocity;
        this.spinAngleScaleFactor = 180.0 / (endAngle - startAngle);
        previousRightVelocity = initialLinearVelocity;
        previousLeftVelocity = initialLinearVelocity;
        mp = new MotionProfileBoxCar(startAngle, endAngle, maxTurnVelocity, itp, t1, t2);
	} 
	
	public MotionProfileTurnPoint getNextPoint(MotionProfileTurnPoint point) {       
        mpPoint = mp.getNextPoint(mpPoint);
        if (mpPoint == null) {
            return null;
        }
        
        point.time = mpPoint.time;
        point.position = mpPoint.position;
        point.velocity = mpPoint.velocity;
        point.acceleration = mpPoint.acceleration;
        
        // Arbitrarily set linear velocity to Vi * Cos(theta) then calculate Vl and Vr.
        if (point != null) {
            double linearVelocity = initialLinearVelocity * Math.cos(Math.toRadians(point.position * spinAngleScaleFactor));
            double angularVelocity = Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor  * Math.toRadians(point.velocity);
            point.rightVelocity = linearVelocity + angularVelocity;
            point.leftVelocity = linearVelocity - angularVelocity;
            point.rightAcceleration = (point.rightVelocity - previousRightVelocity) / mp.getItp() * 1000;
            point.leftAcceleration = (point.leftVelocity - previousLeftVelocity) / mp.getItp() * 1000;
            previousRightVelocity = point.rightVelocity;
            previousLeftVelocity = point.leftVelocity;
        }
		
		return point;
    }
    
    private static double inchesPerSecondToTicksPer100ms(double inches_s) {
		return inchesToRotations(inches_s) * 4096.0 / 10.0;
	}

    public static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	public static void main(String[] args) {
        PrintWriter out = null;
        try {
            out = new PrintWriter("C:/Users/Brian Selle/turn180.csv");
            long startTime = System.nanoTime();

            MotionProfileSpinMove mp = new MotionProfileSpinMove(-48, 0, 180, 180, 1, 200, 100);
            System.out.println("Time Constructor = " +  (System.nanoTime() - startTime) * 1E-6 + " ms");
            out.println("Time, Theta, ThetaDot, LeftVelocity, RightVelocity");
            MotionProfileTurnPoint point = new MotionProfileTurnPoint();
            while (mp.getNextPoint(point) != null) {
                out.println(point.time + ", " + point.position + ", " + point.velocity + ", " + inchesPerSecondToTicksPer100ms(point.leftVelocity) + ", " + inchesPerSecondToTicksPer100ms(point.rightVelocity));
            }

            long deltaTime = System.nanoTime() - startTime;
            System.out.println("Time Box Car = " + (double) deltaTime * 1E-6 + " ms");
        } 
        catch (Exception e) {
            System.out.println("Error in mp turn calc " + e.getMessage());
        }
        finally {
            out.close();
        }
    }
}