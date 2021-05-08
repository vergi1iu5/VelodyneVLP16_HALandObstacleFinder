// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package TerrainAnalysis;

import java.lang.String;
import java.text.DecimalFormat;
/**
 * CreaterObstacle class which extends the Obstacle class. It is meant to be used alongside BoulderObstacle.
 * Both Obstacle types behave similarly except for how we draw the boundary box around it. For a creater, the
 * boundary box is drawn arround the perimeter of the creater. For a bourder, we use it's height as the height 
 * box (meaning the box is vertical).
 * 
 * <p>getterPoint() functions are called by other programs to outline boundary box surrounding obstacle
 * <p>update() functions are called to update the bounds and height/depth of Obstacle depending on the new found point.
 * <p>isSameAs() function is used to compare two obstacles and determe two Obstacles and determine if they should be combined or not
 */
public class CreaterObstacle extends Obstacle{

    private final int x_cord = 0;
    private final int y_cord = 1;
    private final int z_cord = 2;

    private double[] leftMostPoint;
    private double[] rightMostPoint;
    private double[] fardestPoint;
    private double[] closestPoint;
    private double groundRef;
    private obstacleType type;
    private double[] centerCoords;
    private double height;
    /**
     * CreaterObstacle constructor. Assigns type as CREATER and updates boundary box to the 
     * initializer point. Use this function to initialize in cartician coordinates.
     * 
     * @param coords Initializer point. First point to be found as a possible Obstacle.
     * @param groundRefIn Point to be used as the reference for ground.
     */
    public CreaterObstacle(double[] coords, double groundRefIn){
        leftMostPoint = coords.clone();
        rightMostPoint = coords.clone();
        fardestPoint = coords.clone();
        closestPoint = coords.clone();
        height = coords[z_cord];
        centerCoords = coords.clone();
        type = obstacleType.CREATER;
        groundRef = groundRefIn;
    }
    /**
     * CreaterObstacle constructor. Assigns type as CREATER and updates boundary box to the initializer point.
     * Use this function to initialize in polar cordinates which are then turned to cartician.
     * 
     * @param azimuth Azimuth angle, in degrees, at which the obstacle is first found.
     * @param elevation Elevation angle, in degreees, at which the obstacle is first found.
     * @param distance Distance, in meters, at which the obstacle is first found.
     * @param groundRefIn Point to be used as the reference for ground.
     */
    public CreaterObstacle(int azimuth, int elevation, double distance, double groundRefIn){
        double[] coords = toCartesian(azimuth, elevation, distance);
        leftMostPoint = coords.clone();
        rightMostPoint = coords.clone();
        fardestPoint = coords.clone();
        closestPoint = coords.clone();
        height = coords[z_cord];
        centerCoords = coords.clone();
        type = obstacleType.CREATER;
        groundRef = groundRefIn; 
    }
    /**
     * Undate the center coordinates by taking the middle of the boundary box.
     */
    public void updateCenterCoords(){
        centerCoords[x_cord] = (rightMostPoint[x_cord] + leftMostPoint[x_cord]) / 2;
        centerCoords[y_cord] = (fardestPoint[y_cord] + closestPoint[y_cord]) / 2;
        centerCoords[z_cord] = (height - groundRef) / 2;
    }
    /**
     * Get the area of the boundary box surrounding the Obstacle.
     * 
     * @return Area of Obstacle as a double and in m^2
     */
    public double getArea(){
        double width = rightMostPoint[x_cord] - leftMostPoint[x_cord];
        double length = fardestPoint[y_cord] - closestPoint[y_cord];
        return length * width;
    }
    /**
     * Use a new found point to update the boundary of the box. The box is extended
     * to emcompass the new point.
     * 
     * @param coords New found point
     */
    public void updateBounds(double[] coords){
        //Determine if the center needs to be updated based on if the 
        //boundary box gets updated
        boolean centerUpdateNeeded = false;

        if(leftMostPoint[x_cord] > coords[x_cord]){
            centerUpdateNeeded = true;
            leftMostPoint[x_cord] = coords[x_cord];
            leftMostPoint[y_cord] = coords[y_cord];
            leftMostPoint[z_cord] = coords[z_cord];
        }
        if(rightMostPoint[x_cord] < coords[x_cord]){
            rightMostPoint[x_cord] = coords[x_cord];
            rightMostPoint[y_cord] = coords[y_cord];
            rightMostPoint[z_cord] = coords[z_cord];
        }
        if(fardestPoint[y_cord] < coords[y_cord]){
            centerUpdateNeeded = true;
            fardestPoint[x_cord] = coords[x_cord];
            fardestPoint[y_cord] = coords[y_cord];
            fardestPoint[z_cord] = coords[z_cord];
        }
        if(closestPoint[y_cord] > coords[y_cord]){
            centerUpdateNeeded = true;
            closestPoint[x_cord] = coords[x_cord];
            closestPoint[y_cord] = coords[y_cord];
            closestPoint[z_cord] = coords[z_cord];
        }
        if(height > coords[z_cord]){
            centerUpdateNeeded = true;
            height = coords[z_cord];
        }
        //Update center if needed
        if(centerUpdateNeeded){
            updateCenterCoords();
        }

    }
    /**
     * Update the boundary box with a new found point but in polar cordinates. The
     * coordinates are turned to cartician first.
     * 
     * @param azimuth Azimuth angle, in degrees, at which the obstacle is found
     * @param elevation Elevation angle, in degrees, at which the obstacle is found
     * @param distance Distance, in meters, at which the obstacle is found 
     */
    public void updateBounds(int azimuth, int elevation, double distance){
        double[] coords = toCartesian(azimuth, elevation, distance);
        updateBounds(coords);
    }
    /**
     * If two obstacles are found to have intersecting boundary boxes, then combine
     * both obstacles into a single one.
     * 
     * @param o Obstacle to be combined with.
     */
    public void combineObstacles(Obstacle o){

        updateBounds(o.getLeftMostPoint());
        updateBounds(o.getRightMostPoint());
        updateBounds(o.getFardestPoint());
        updateBounds(o.getClosestPoint());

    }
    /**
     * Get the type of Obstacle
     * 
     * @return Enumerator type inidicating a CreaterObstacle;
     */
    public obstacleType getType(){
        return type;
    }
    
    public double[] getLeftMostPoint(){
        return leftMostPoint.clone();
    }
    
    public double[] getRightMostPoint(){
        return rightMostPoint.clone();
    }
    
    public double[] getFardestPoint(){
        return fardestPoint.clone();
    }
    
    public double[] getClosestPoint(){
        return closestPoint.clone();
    }
    /**
     * Determine if two Obstacles are the same by seeing if their boundary boxes intersect.
     * 
     * @return Boolean value, True if the two obstacles are the same.
     */
    public boolean isSameAs(Obstacle o){
        //Not the same if they are not the same type
        if(this.type != o.getType()){
            return false;
        }
        //Obstacle's left most point is within the bounds of self's box
        if(rightMostPoint[x_cord] > o.getLeftMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getLeftMostPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getLeftMostPoint()[y_cord] && closestPoint[y_cord] < o.getLeftMostPoint()[y_cord]){
                return true;
            }
        }
        //Obstacle's right most point is within the bounds of self's box
        if(rightMostPoint[x_cord] > o.getRightMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getRightMostPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getRightMostPoint()[y_cord] && closestPoint[y_cord] < o.getRightMostPoint()[y_cord]){
                return true;
            }
        }
        //Obstacle's fardest point is within the bounds of self's box
        if(rightMostPoint[x_cord] > o.getFardestPoint()[x_cord] && leftMostPoint[x_cord] < o.getFardestPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getFardestPoint()[y_cord] && closestPoint[y_cord] < o.getFardestPoint()[y_cord]){
                return true;
            }
        }
        //Obstacle's closest point is within the bounds of self's box
        if(rightMostPoint[x_cord] > o.getClosestPoint()[x_cord] && leftMostPoint[x_cord] < o.getClosestPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getClosestPoint()[y_cord] && closestPoint[y_cord] < o.getClosestPoint()[y_cord]){
                return true;
            }
        }
        
        return false;
    }
    /**
     * Turn Creater obstacle into a string for debugging:
     * 
     * Obstacle Type:
     *  X:
     *  Y:
     *  Z:
     *  Depth:
     * @return String detailing obstacle for debugging
     */
    public String toString(){
      String s = "Creater";
      
      DecimalFormat df = new DecimalFormat("#.##");
      
      String ret = "Obstacle Type: " + s + "\n";
      ret += "\tCentered at:\n";
      ret += "\t\tX: " + df.format(this.centerCoords[0]) + "\n";
      ret += "\t\tY: " + df.format(this.centerCoords[1]) + "\n";
      ret += "\t\tZ: " + df.format(this.centerCoords[2]) + "\n";
      ret += "\t\tDepth: " + df.format(this.height);
      return ret;
    }
}
