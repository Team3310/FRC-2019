package frc.team3310.utility;

import java.io.PrintWriter;

import frc.team3310.robot.Constants;

public class MotionProfileSpinMove {

    private MotionProfileBoxCar mp;
    private double initialLinearVelocity;
    private double spinAngleScaleFactor;

	public MotionProfileSpinMove(double initialLinearVelocity, double startAngle, double endAngle, double maxTurnVelocity, double itp, double t1, double t2) {
        this.initialLinearVelocity = initialLinearVelocity;
        this.spinAngleScaleFactor = 180.0 / (endAngle - startAngle);
        mp = new MotionProfileBoxCar(startAngle, endAngle, maxTurnVelocity, itp, t1, t2);
	} 
	
	public MotionProfileTurnPoint getNextPoint(MotionProfileTurnPoint point) {
		
        point = (MotionProfileTurnPoint)mp.getNextPoint(point);
        
        // Arbitrarily set linear velocity to Vi * Cos(theta) then calculate Vl and Vr.
        if (point != null) {
            double linearVelocity = initialLinearVelocity * Math.cos(Math.toRadians(point.position * spinAngleScaleFactor));
            double angularVelocity = Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor  * Math.toRadians(point.velocity);
            point.rightVelocity = linearVelocity + angularVelocity;
            point.leftVelocity = linearVelocity - angularVelocity;
        }
		
		return point;
	}

	public static void main(String[] args) {
        PrintWriter out = null;
        try {
            out = new PrintWriter("C:/Users/Brian Selle/turn180.csv");
            long startTime = System.nanoTime();

            MotionProfileSpinMove mp = new MotionProfileSpinMove(-48, 0, 90, 180, 1, 200, 100);
            System.out.println("Time Constructor = " +  (System.nanoTime() - startTime) * 1E-6 + " ms");
            out.println("Time, Theta, ThetaDot, LeftVelocity, RightVelocity");
            MotionProfileTurnPoint point = new MotionProfileTurnPoint();
            while (mp.getNextPoint(point) != null) {
                out.println(point.time + ", " + point.position + ", " + point.velocity + ", " + point.leftVelocity + ", " + point.rightVelocity);
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