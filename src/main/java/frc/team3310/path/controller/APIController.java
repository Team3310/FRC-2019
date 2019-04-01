package frc.team3310.path.controller;

import java.lang.reflect.Field;
import java.net.URLDecoder;
import java.util.ArrayList;

import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;
import org.springframework.web.bind.annotation.RestController;

import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.paths.TrajectoryGenerator.TrajectorySet;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;
import frc.team3310.utility.lib.spline.QuinticHermiteSpline;
import frc.team3310.utility.lib.spline.Spline;
import frc.team3310.utility.lib.spline.SplineGenerator;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.MirroredTrajectory;
import frc.team3310.utility.lib.trajectory.Trajectory;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

@RestController
@RequestMapping("api")
public class APIController {

    @RequestMapping(value = "/calculate_splines", method = RequestMethod.POST)
    public @ResponseBody String calcSplines(@RequestBody String message) {
        message = message.substring(0, message.length() - 1);

        try {
            message = URLDecoder.decode(message, "UTF-8");
        } catch (Exception e) {
            e.printStackTrace();
        }

        ArrayList<Pose2d> points = new ArrayList<>();
        for (String pointString : message.split(";")) {
            String[] pointData = pointString.split(",");

            int x = pointData[0].equals("NaN") ? 0 : Integer.parseInt(pointData[0]);
            int y = pointData[1].equals("NaN") ? 0 : Integer.parseInt(pointData[1]);
            int heading = pointData[2].equals("NaN") ? 0 : Integer.parseInt(pointData[2]);

            points.add(new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(heading)));
        }

        return pointsToSpline(points);
    }

    @RequestMapping(value = "/get_trajectory", method = RequestMethod.POST)
    public @ResponseBody String getTrajectory(@RequestBody String trajectoryName) {
        trajectoryName = trajectoryName.substring(0, trajectoryName.length() - 1);

        System.out.println("Loading Trajectory = " + trajectoryName);

        try {
            trajectoryName = URLDecoder.decode(trajectoryName, "UTF-8");
        } catch (Exception e) {
            e.printStackTrace();
        }

        TrajectoryGenerator trajectories = TrajectoryGenerator.getInstance();
        trajectories.generateTrajectories();

        Trajectory<TimedState<Pose2dWithCurvature>> trajectory = null;
        try {
            boolean isLeft = false;
            if (trajectoryName.endsWith("_Left")) {
                trajectoryName = trajectoryName.substring(0, trajectoryName.length() - 5);
                isLeft = true;
            }
            else if (trajectoryName.endsWith("_Right")) {
                trajectoryName = trajectoryName.substring(0, trajectoryName.length() - 6);
                isLeft = false;
            }

            Field field = TrajectorySet.class.getField(trajectoryName);

            LazyLoadTrajectory lazyLoadTrajectory = (LazyLoadTrajectory)field.get(trajectories.getTrajectorySet());
            lazyLoadTrajectory.activate();

            MirroredTrajectory mirroredTrajectory = lazyLoadTrajectory.getTrajectory();
            trajectory = mirroredTrajectory.get(isLeft);
        }
        catch (Exception e) {
            System.out.println("Unable to find trajectory = " + trajectoryName);
        }
        
        ArrayList<Pose2dWithCurvature> positions = new ArrayList<>();
        for (int i = 0; i < trajectory.length(); i++) {
            positions.add(trajectory.getState(i).state());
        }

        return trajectoryToJson(positions);
    }

    @RequestMapping(value = "/get_trajectory_list", method = RequestMethod.POST)
    public @ResponseBody String getTrajectoryList(@RequestBody String message) {
        TrajectoryGenerator trajectories = TrajectoryGenerator.getInstance();
        trajectories.generateTrajectories();

        StringBuilder strBuilder = new StringBuilder();
        strBuilder.append("{\"trajectories\":[");

        Field[] allFields = TrajectorySet.class.getDeclaredFields();
        for (Field field : allFields) {
            if (field.getType().equals(LazyLoadTrajectory.class)) {
                strBuilder.append("\"" + field.getName() + "_Left" + "\",");
                strBuilder.append("\"" + field.getName() + "_Right" + "\",");
            }
            else if (field.getType().equals(Trajectory.class)) {
                strBuilder.append("\"" + field.getName() + "\",");
            }
        }

        return strBuilder.substring(0, strBuilder.length() - 1) + "]}";
    }

    private String pointsToSpline(ArrayList<Pose2d> points) {
        ArrayList<QuinticHermiteSpline> mQuinticHermiteSplines = new ArrayList<>();
        ArrayList<Spline> mSplines = new ArrayList<>();
        ArrayList<Pose2dWithCurvature> positions = new ArrayList<>();
        if (points.size() < 2) {
            return "no";
        } else {
            for (int i = 0; i < points.size() - 1; i++) {
                mQuinticHermiteSplines.add(new QuinticHermiteSpline(points.get(i), points.get(i + 1)));
            }

            QuinticHermiteSpline.optimizeSpline(mQuinticHermiteSplines);

            for (QuinticHermiteSpline mQuinticHermiteSpline : mQuinticHermiteSplines) {
                mSplines.add(mQuinticHermiteSpline);
            }

            positions.addAll(SplineGenerator.parameterizeSplines(mSplines));
        }

        return trajectoryToJson(positions);
    }

    private String trajectoryToJson(ArrayList<Pose2dWithCurvature> positions) {
        String json = "{\"points\":[";
        for (Pose2dWithCurvature pose : positions) {
            json += poseToJSON(pose) + ",";
        }

        return json.substring(0, json.length() - 1) + "]}";
    }

    private String poseToJSON(Pose2dWithCurvature pose) {
        double x = pose.getTranslation().x();
        double y = pose.getTranslation().y();
        double rotation = pose.getRotation().getRadians();
        double curvature = pose.getCurvature();

        return "{\"x\":" + x + ", \"y\":" + y + ", \"rotation\":" + rotation + ", \"curvature\":" + curvature + "}";
    }
}