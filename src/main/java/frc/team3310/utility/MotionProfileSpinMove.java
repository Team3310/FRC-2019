package frc.team3310.utility;

import java.io.PrintWriter;

public class MotionProfileSpinMove {

    private MotionProfileBoxCar mp;
    private double initialLinearVelocity;
    private double spinAngleScaleFactor;
    private boolean flip = false;

	public MotionProfileSpinMove(double initialLinearVelocity, double startAngle, double endAngle, double maxTurnVelocity, double itp, double t1, double t2, boolean flip) {
        this.initialLinearVelocity = initialLinearVelocity;
        this.spinAngleScaleFactor = 180.0 / (endAngle - startAngle);
        this.flip = flip;
        mp = new MotionProfileBoxCar(startAngle, endAngle, maxTurnVelocity, itp, t1, t2);
	} 
	
	public MotionProfileTurnPoint getNextPoint(MotionProfileTurnPoint point) {
		
        point = (MotionProfileTurnPoint)mp.getNextPoint(point);
        
        if (point != null) {
            double angleRadians = Math.toRadians(point.position * spinAngleScaleFactor);
            double cosA = Math.cos(angleRadians);
            double sinA = Math.sin(angleRadians);
            if (flip) {
                sinA = -sinA;
            }
            point.rightVelocity = initialLinearVelocity * (cosA + sinA);
            point.leftVelocity = initialLinearVelocity * (cosA - sinA);
        }
		
		return point;
	}

	public static void main(String[] args) {
        PrintWriter out = null;
        try {
            out = new PrintWriter("C:/Users/Brian Selle/turn180.csv");
            long startTime = System.nanoTime();

            MotionProfileSpinMove mp = new MotionProfileSpinMove(1, 0, 180, 360, 10, 200, 100, false);
            out.println("Time, Position, Velocity, LeftVelocity, RightVelocity");
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