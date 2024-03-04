package frc.lib.Utils;

public class Util1785 {
    /**
     * 
     * @param yaw angle in degrees
     * @param trackedDistance distance from target in meters
     * @param camDistance distance from camera to center in meters
     * @return
     */
    public static double getRobotRelativeAngle(double yaw, double trackedDistance, double camDistance){
        return 90 - 180 / Math.PI * Math.atan( (trackedDistance * Math.cos( yaw * Math.PI / 180)) / (trackedDistance * Math.sin(yaw * Math.PI/180) - camDistance) );
    }

    public static double getDistanceRobotRelative(double yaw, double trackedDistance, double camDistance){
        return Math.sqrt(Math.pow(trackedDistance * Math.cos(yaw * Math.PI / 180) , 2) + Math.pow(trackedDistance * Math.sin(yaw * Math.PI / 180) - camDistance , 2));
    }

}
