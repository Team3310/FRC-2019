package frc.team3310.test.java.com.team3310.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;
import frc.team3310.utility.lib.spline.QuinticHermiteSpline;
import frc.team3310.utility.Util;

public class QuinticHermiteOptimizerTest {
    private static double kEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        Pose2d a = new Pose2d(new Translation2d(0, 100), Rotation2d.fromDegrees(270));
        Pose2d b = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d c = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(90));

        List<QuinticHermiteSpline> splines = new ArrayList<>();
        splines.add(new QuinticHermiteSpline(a, b));
        splines.add(new QuinticHermiteSpline(b, c));

        long startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines) < 0.014);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2d d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        Pose2d e = new Pose2d(new Translation2d(0, 50), Rotation2d.fromDegrees(0));
        Pose2d f = new Pose2d(new Translation2d(100, 0), Rotation2d.fromDegrees(90));
        Pose2d g = new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0));

        List<QuinticHermiteSpline> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermiteSpline(d, e));
        splines1.add(new QuinticHermiteSpline(e, f));
        splines1.add(new QuinticHermiteSpline(f, g));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines1) < 0.16);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));


        Pose2d h = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        Pose2d i = new Pose2d(new Translation2d(50, 0), Rotation2d.fromDegrees(0));
        Pose2d j = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(45));
        Pose2d k = new Pose2d(new Translation2d(150, 0), Rotation2d.fromDegrees(270));
        Pose2d l = new Pose2d(new Translation2d(150, -50), Rotation2d.fromDegrees(270));

        List<QuinticHermiteSpline> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermiteSpline(h, i));
        splines2.add(new QuinticHermiteSpline(i, j));
        splines2.add(new QuinticHermiteSpline(j, k));
        splines2.add(new QuinticHermiteSpline(k, l));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines2) < 0.05);
        assertEquals(splines2.get(0).getCurvature(1.0), 0.0, kEpsilon);
        assertEquals(splines2.get(2).getCurvature(1.0), 0.0, kEpsilon);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }
}
