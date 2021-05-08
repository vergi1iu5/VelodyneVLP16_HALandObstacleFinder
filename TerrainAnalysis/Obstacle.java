// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package com.AURMC.lib.TerrainAnalysis;

import com.AURMC.lib.Hardware.VelodyneLidarHDL.PacketDecoder;
import java.lang.String;
/**
 * Abstract class to outline how an Obstacle classs should behave and be seen by the rest of the mapping and pathing program. 
 * There are two types: Boulders or Creaters with a negative height indicating a Creater.
 * 
 * <p>getterPoint() functions are called by other programs to outline boundary box surrounding obstacle
 * <p>update() functions are called to update the bounds and height/depth of Obstacle depending on the new found point.
 * <p>isSameAs() function is used to compare two obstacles and determe two Obstacles and determine if they should be combined or not
 */
public abstract class Obstacle {

    private double LIDAR_HIGHT_M = 0.381;
    /**
     * Enumerator to indicate type of Obstacle object
     */
    public enum obstacleType{
        NONE,    /**<Default type */
        BOULDER, /**<Obstacle with a height greater than zero */
        CREATER; /**<Obstacle with a height less than zero */
    }   
    /**
     * Call function to update center coordinates based on the current bounds
     */
    public abstract void updateCenterCoords();
    /**
     * Call function to get the area of the boundary box surrounding the Obstacle
     * 
     * @return Area of boundary box surrounding the Obtacle.
     */
    public abstract double getArea();
    /**
     * Feed in a point represented as a double array which is determine to be part of the Obstacle.
     * The function then uses this new found point to update boundary box.
     * 
     * @param coords point as a double array.
     */
    public abstract void updateBounds(double[] coords);
    /**
     * Update the boundary box of an Obstacle by using polar coordinates. The fed in values will then 
     * be used to calculate the pertaining point in cartician. 
     * 
     * @param azimuth azimuth angle (degrees) as an integer and multiplied by 100 (ex: 10.20 -> 1020)
     * @param elevation elevation angle (degrees) as an integer and multiplied by 100 (ex: 10.20 -> 1020)
     * @param distance distance in meters as a double
     */
    public abstract void updateBounds(int azimuth, int elevation, double distance);
    /**
     * Get the type of Obstacle
     * 
     * @return obstacleType enumerator
     */
    public abstract obstacleType getType();
    /**
     * Get the left most point of the boundary box surrounding the obstacle
     * 
     * @return double array containing the X, Y, and Z coordinates of the left most boundary point
     */    
    public abstract double[] getLeftMostPoint();
        /**
     * Get the right most point of the boundary box surrounding the obstacle
     * 
     * @return double array containing the X, Y, and Z coordinates of the right most boundary point
     */     
    public abstract double[] getRightMostPoint();
    /**
     * Get the farthest (from rover) point of the boundary box surrounding the obstacle
     * 
     * @return double array containing the X, Y, and Z coordinates of the farthest (from rover) boundary point
     */      
    public abstract double[] getFardestPoint();
    /**
     * Get the closest (to rover) point of the boundary box surrounding the obstacle
     * 
     * @return double array containing the X, Y, and Z coordinates of the closest (to rover) boundary point
     */     
    public abstract double[] getClosestPoint();
    /**
     * Call function on an Obstacle to compare it to another Obstacle. Obstacles are
     * compared based on their type and boundary boxes.
     * 
     * @param o Obstacle to compare Self to
     * @return Boolean value indicating if the two Obstacles match
     */
    public abstract boolean isSameAs(Obstacle o);
    /**
     * Call this function to combine self with another Obstacle. This function
     * expands the boundary box of self and destroys the other Object.
     * 
     * @param o Obstacle to combine with
     */
    public abstract void combineObstacles(Obstacle o);
    /**
     * Function to retreive a string detailing the Obstacle.
     * Type:
     * Center X:
     * Center Y:
     * Center Z:
     * 
     * @return String detailing Obstacle
     */
    public abstract String toString();
    /**
     * Turn given polar coordinates into cartician coordinates based on the laser Id, distance and azimuth values
     * 
     * @param azimuth
     * @param laserID
     * @param distance
     * @return Double array containing {X,Y,Z}
     */
    protected double[] toCartesian(int azimuth, int laserID, double distance){
        double cosAz = PacketDecoder.Az_cos_lookup_table[azimuth];
        double sinAz = PacketDecoder.Az_sin_lookup_table[azimuth];
        double cosEl = PacketDecoder.El_cos_lookup_table[laserID % 16][0];
        double sinEl = PacketDecoder.El_sin_lookup_table[laserID % 16][0];

        double xy_plane_projection = distance * cosEl;
        double X = xy_plane_projection * sinAz;
        double Y = xy_plane_projection * cosAz;
        double Z = distance * sinEl + PacketDecoder.Lidar_height_map[azimuth];

        return new double[] {X, Y, Z};

    }
}
