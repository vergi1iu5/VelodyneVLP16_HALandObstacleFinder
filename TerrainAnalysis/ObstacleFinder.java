// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package com.AURMC.lib.TerrainAnalysis;

import com.AURMC.lib.Hardware.VelodyneLidarHDL.PacketDecoder.HDLFrame;

import java.util.ArrayList;
import java.util.Queue;
import java.util.LinkedList;
import java.lang.String;
import java.text.DecimalFormat;
/**
 * ObstacleFinder implements TerrainAnalyzer to be used in tandum with PackedDecorder's output frames and the
 * two types of Obstacles, Boulder and Creater.
 * 
 * <p>findObstaclesPolar() takes in an HDLFrame and analyzes it in polar coordinates where the distance to a sampled point 
 * is compared to that of a flat plane.
 * <p>findObstaclesCartician() takes in an HDLFrame and analyzes it in cartician coordinates where it looks for concistant 
 * changes in height.
 * <p>Obstacle() related functions are then used to get number and retreive obstacles found if any.
 */
public class ObstacleFinder implements TerrainAnalyzer {

    private final int HDL_LASER_PER_FIRING = 32;
    private final int x_cord = 0;
    private final int y_cord = 1;
    private final int z_cord = 2;

    private static double _heightTolarence;
    private static double _groundRef;
    private static int _positiveHitsThreshold;
    private ArrayList<Obstacle> _foundObstacles = new ArrayList<Obstacle>();

    public class CreaterObstacle extends Obstacle{

        private double[] leftMostPoint;
        private double[] rightMostPoint;
        private double[] fardestPoint;
        private double[] closestPoint;
        private double groundRef;
        private obstacleType type;
        private double[] centerCoords;
        private double height;
        
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

        public CreaterObstacle(int azimuth, int elevation, double distance, double groundRefIn){
            //azimuth = 9000 - azimuth;
            //int az = (azimuth < 0) ? (36000 - (Math.abs(azimuth) % 36000) ) % 36000: (azimuth % 36000);
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

        public void updateCenterCoords(){
            centerCoords[x_cord] = (rightMostPoint[x_cord] + leftMostPoint[x_cord]) / 2;
            centerCoords[y_cord] = (fardestPoint[y_cord] + closestPoint[y_cord]) / 2;
            centerCoords[z_cord] = (height - groundRef) / 2;
        }

    
        public double getArea(){
            double width = rightMostPoint[x_cord] - leftMostPoint[x_cord];
            double length = fardestPoint[y_cord] - closestPoint[y_cord];
            return length * width;
        }
    
        public void updateBounds(double[] coords){

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

            if(centerUpdateNeeded){
                updateCenterCoords();
            }

        }

        public void updateBounds(int azimuth, int elevation, double distance){
            //azimuth = 9000 - azimuth;
            //int az = (azimuth < 0) ? (36000 - (Math.abs(azimuth) % 36000) ) % 36000: (azimuth % 36000);
            double[] coords = toCartesian(azimuth, elevation, distance);
            updateBounds(coords);
        }

        public void combineObstacles(Obstacle o){

            updateBounds(o.getLeftMostPoint());
            updateBounds(o.getRightMostPoint());
            updateBounds(o.getFardestPoint());
            updateBounds(o.getClosestPoint());

        }

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
    
        public boolean isSameAs(Obstacle o){
            if(this.type != o.getType()){
                return false;
            }

            if(rightMostPoint[x_cord] > o.getLeftMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getLeftMostPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getLeftMostPoint()[y_cord] && closestPoint[y_cord] < o.getLeftMostPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getRightMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getRightMostPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getRightMostPoint()[y_cord] && closestPoint[y_cord] < o.getRightMostPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getFardestPoint()[x_cord] && leftMostPoint[x_cord] < o.getFardestPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getFardestPoint()[y_cord] && closestPoint[y_cord] < o.getFardestPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getClosestPoint()[x_cord] && leftMostPoint[x_cord] < o.getClosestPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getClosestPoint()[y_cord] && closestPoint[y_cord] < o.getClosestPoint()[y_cord]){
                    return true;
                }
            }
            
            return false;
        }
        
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

    public class BoulderObstacle extends Obstacle{

        private double[] leftMostPoint;
        private double[] rightMostPoint;
        private double[] fardestPoint;
        private double[] closestPoint;
        private double groundRef;
        private obstacleType type;
        private double[] centerCoords;
        private double height;
        
        public BoulderObstacle(double[] coords, double groundRefIn){
            leftMostPoint = coords.clone();
            rightMostPoint = coords.clone();
            fardestPoint = coords.clone();
            closestPoint = coords.clone();
            height = coords[z_cord];
            centerCoords = coords.clone();
            type = obstacleType.BOULDER;
            groundRef = groundRefIn;
        }

        public BoulderObstacle(int azimuth, int elevation, double distance, double groundRefIn){
            //azimuth = 9000 - azimuth;
            //int az = (azimuth < 0) ? (36000 - (Math.abs(azimuth) % 36000) ) % 36000: (azimuth % 36000);
            double[] coords = toCartesian(azimuth, elevation, distance);
            leftMostPoint = coords.clone();
            rightMostPoint = coords.clone();
            fardestPoint = coords.clone();
            closestPoint = coords.clone();
            height = coords[z_cord];
            centerCoords = coords.clone();
            groundRef = groundRefIn;
            type = obstacleType.BOULDER;
        }

        public void updateCenterCoords(){
            centerCoords[x_cord] = (rightMostPoint[x_cord] + leftMostPoint[x_cord]) / 2;
            centerCoords[y_cord] = (fardestPoint[y_cord] + closestPoint[y_cord]) / 2;
            centerCoords[z_cord] = (height - groundRef) / 2;
        }
    
        public double getArea(){
            double width = rightMostPoint[x_cord] - leftMostPoint[x_cord];
            return height * width;
        }
    
        public void updateBounds(double[] coords){

            boolean centerUpdateNeeded = false;
            if(leftMostPoint[x_cord] > coords[x_cord]){
                centerUpdateNeeded = true;
                leftMostPoint[x_cord] = coords[x_cord];
                leftMostPoint[y_cord] = coords[y_cord];
                leftMostPoint[z_cord] = coords[z_cord];
            }
            if(rightMostPoint[x_cord] < coords[x_cord]){
               centerUpdateNeeded = true;
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
            if(this.height < coords[z_cord]){
                centerUpdateNeeded = true;
                height = coords[z_cord];
            }

            if(centerUpdateNeeded){
                updateCenterCoords();
            }

        }

        public void updateBounds(int azimuth, int elevation, double distance){
            //azimuth = 9000 - azimuth;
            //int az = (azimuth < 0) ? (36000 - (Math.abs(azimuth) % 36000) ) % 36000: (azimuth % 36000);
            double[] coords = toCartesian(azimuth, elevation, distance);
            updateBounds(coords);
        }

        public void combineObstacles(Obstacle o){

            updateBounds(o.getLeftMostPoint());
            updateBounds(o.getRightMostPoint());
            updateBounds(o.getFardestPoint());
            updateBounds(o.getClosestPoint());

        }

        public obstacleType getType(){
            return this.type;
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
        
        public boolean isSameAs(Obstacle o){
            if(type != o.getType()){
                return false;
            }

            if(rightMostPoint[x_cord] > o.getLeftMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getLeftMostPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getLeftMostPoint()[y_cord] && closestPoint[y_cord] < o.getLeftMostPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getRightMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getRightMostPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getRightMostPoint()[y_cord] && closestPoint[y_cord] < o.getRightMostPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getFardestPoint()[x_cord] && leftMostPoint[x_cord] < o.getFardestPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getFardestPoint()[y_cord] && closestPoint[y_cord] < o.getFardestPoint()[y_cord]){
                    return true;
                }
            }
            
            if(rightMostPoint[x_cord] > o.getClosestPoint()[x_cord] && leftMostPoint[x_cord] < o.getClosestPoint()[x_cord]){
                if(fardestPoint[y_cord] > o.getClosestPoint()[y_cord] && closestPoint[y_cord] < o.getClosestPoint()[y_cord]){
                    return true;
                }
            }
            
            return false;
        }
        
        public String toString(){
          String s = "Boulder";
          
          DecimalFormat df = new DecimalFormat("#.##");
          
          String ret = "Obstacle Type: " + s + "\n";
          ret += "\tCentered at:\n";
          ret += "\t\tX: " + df.format(this.centerCoords[0]) + "\n";
          ret += "\t\tY: " + df.format(this.centerCoords[1]) + "\n";
          ret += "\t\tZ: " + df.format(this.centerCoords[2]) + "\n";
          ret += "\t\tHeight: " + df.format(this.height);
          return ret;
      }
    }
    
    private HDLFrame _frame; /**<Frame to be analyzed for Obstacles */
    /**
     * ObstacleFinder constructor.
     * 
     * @param heightTolarence Threshold, in meters, in either height (cartician) or distance (polar) to trigger a search
     * @param groundRef Referance point, in meters, to be used as ground
     * @param positiveHitsThreshold Number of possitive hits/points found to declare an Obstacle found
     */
    public ObstacleFinder(double heightTolarence, double groundRef, int positiveHitsThreshold){
        _heightTolarence = heightTolarence;
        _groundRef = groundRef;
        _positiveHitsThreshold = positiveHitsThreshold;
    }
    /**
     * Look for obstacles in polar coordinates within the provided frame. Found Obstacles are added to ArrayList and retreived through
     * getters.
     * 
     * @param frame HDLFrame used to look for obstacles. To work correctly, the frame has to have been produced by a calibrated decoder/lidar.
     * Refer to PacketDecoder and VelodyneLidar class for more info with regards to HDLFrame
     */
    public void findObstaclesPolar(HDLFrame frame){
        _frame = frame;
        int number_of_azimuths = frame.getNumberOfAzimuthsInFrame();
        double[][][] distances = new double[HDL_LASER_PER_FIRING / 2][number_of_azimuths][3];
        frame.getSortedDistances(distances, number_of_azimuths);

        for(int laserID = 0; laserID < HDL_LASER_PER_FIRING / 2; laserID++){
            for(int azimuthID = 0; azimuthID < number_of_azimuths; azimuthID++){
                if((java.lang.Math.abs(distances[laserID][azimuthID][1]) > _heightTolarence)){
                    Obstacle obstacle;
                    if((obstacle = this._investigateAreaPolar(distances, laserID, azimuthID, number_of_azimuths)) != null){
                        addObstacle(obstacle);
                    }
                }
            }
        }
    }
    /**
     * Look for Obstacles in cartician coordinates within the provided HDLFrame. Frame does not have to come from a 
     * calibrated decoder/lidar.
     * 
     * @param frame HDLFrame used to look for obstacles.
     * Refer to PacketDecoder and VelodyneLidar class for more info with regards to HDLFrame
     */
    public void findObstaclesCartician(HDLFrame frame){
        int number_of_azimuths = frame.getNumberOfAzimuthsInFrame();
        double[][][] pointCloud = new double[HDL_LASER_PER_FIRING / 2][number_of_azimuths][3];
        frame.getSortedPointCloud(pointCloud, number_of_azimuths);
 
        for(int laserID = 0; laserID < HDL_LASER_PER_FIRING / 2; laserID++){
            for(int azimuthID = 0; azimuthID < number_of_azimuths; azimuthID++){
                double zValue = pointCloud[laserID][azimuthID][z_cord];
                if((java.lang.Math.abs(zValue - _groundRef) > _heightTolarence)){
                    Obstacle obstacle;
                    if((obstacle = this._investigateAreaCartesian(pointCloud, laserID, azimuthID, number_of_azimuths)) != null){
                        addObstacle(obstacle);
                    }
                }
            }
        }
    }
    /**
     * Add a single Obstacle to ArrayList
     * 
     * @param o Obstacle to be added to array
     */
    public void addObstacle(Obstacle o){
        _foundObstacles.add(o);
    }
    /**
     * Once a positive hit is found, use this function to perform a BFS arounf found point. Returns an Obsticle if enough 
     * points around initial find are found.
     * 
     * @param pointCloud Entire 3D array containing the point-cloud data for searching
     * @param laserID   LaserID for initially found point
     * @param azimuthID Azimuth index number for initially found point (not the actual angle, rather the index within 3D array)
     * @param number_of_azimuths Number of azimuths used to create the provided point cloud data.
     * @return  Obstacle found if any, NULL if none found. 
     */
    private Obstacle _investigateAreaCartesian(double[][][] pointCloud, int laserID, int azimuthID, int number_of_azimuths){

        Queue<int[]> searchQueue = new LinkedList<>();
        int foundPoints = 1;
        Obstacle candidate;
        double height = pointCloud[laserID][azimuthID][z_cord];

        if(height < 0){
            candidate = new CreaterObstacle(pointCloud[laserID][azimuthID], _groundRef);
        }else{
            candidate = new BoulderObstacle(pointCloud[laserID][azimuthID], _groundRef);
        }

        searchQueue.add(new int[] {laserID, azimuthID});

        while(!searchQueue.isEmpty()){

            int[] searchCoords = searchQueue.remove();
            laserID = searchCoords[0];
            azimuthID = searchCoords[1];

            if(laserID - 1 >= 0){
                candidate.updateBounds(pointCloud[laserID - 1][azimuthID]);
                if(java.lang.Math.abs(pointCloud[laserID - 1][azimuthID][z_cord] - _groundRef) > _heightTolarence){
                    foundPoints++;
                    pointCloud[laserID - 1][azimuthID][z_cord] = _groundRef;
                    searchQueue.add(new int[] {laserID - 1, azimuthID});
                }
            }
            if(laserID + 1 < HDL_LASER_PER_FIRING / 2){
                candidate.updateBounds(pointCloud[laserID + 1][azimuthID]);
                if(java.lang.Math.abs(pointCloud[laserID + 1][azimuthID][z_cord] - _groundRef) > _heightTolarence){
                    foundPoints++;
                    pointCloud[laserID + 1][azimuthID][z_cord] = _groundRef;
                    searchQueue.add(new int[] {laserID + 1, azimuthID});
                }
            }
            if(azimuthID - 1 >= 0){
                candidate.updateBounds(pointCloud[laserID][azimuthID - 1]);
                if(java.lang.Math.abs(pointCloud[laserID][azimuthID - 1][z_cord] - _groundRef) > _heightTolarence){
                    foundPoints++;
                    pointCloud[laserID][azimuthID - 1][z_cord] = _groundRef;
                    searchQueue.add(new int[] {laserID, azimuthID - 1});
                }
            }
            if(azimuthID + 1 < number_of_azimuths){
                candidate.updateBounds(pointCloud[laserID][azimuthID + 1]);
                if(java.lang.Math.abs(pointCloud[laserID][azimuthID + 1][z_cord] - _groundRef) > _heightTolarence){
                    foundPoints++;
                    pointCloud[laserID][azimuthID + 1][z_cord] = _groundRef;
                    searchQueue.add(new int[] {laserID, azimuthID + 1});
                }
            }    
        }
        if(foundPoints > _positiveHitsThreshold){
            return candidate;
        }else{
            return null;
        }
    }
    /**
     * Once a positive hit is found, use this function to perform a BFS around the found point, using distance as the judging factor.
     * 
     * @param distances 2D array of all sampled points for searching area.
     * @param laserID LaserID for the initially found positive hit
     * @param azimuthID Azimuth index for the initially found positive hit.
     * @param number_of_azimuths Number of azimuths in the searching area (i.e. maximum azimuthID)
     * @return Obstacle object if any found, NULL if not.
     */
    private Obstacle _investigateAreaPolar(double[][][] distances, int laserID, int azimuthID, int number_of_azimuths){

        Queue<int[]> searchQueue = new LinkedList<>();
        int foundPoints = 1;
        Obstacle candidate;
        double height = distances[laserID][azimuthID][1];
        int azimuth = (int) distances[laserID][azimuthID][0];
        if(height < 0){
            candidate = new BoulderObstacle((int) distances[laserID][azimuthID][0], laserID,(double) _frame.getDistance(laserID, azimuth), _groundRef) ;
        }else{
            candidate = new CreaterObstacle((int) distances[laserID][azimuthID][0], laserID,(double) _frame.getDistance(laserID, azimuth), _groundRef);
        }

        searchQueue.add(new int[] {laserID, azimuthID});

        while(!searchQueue.isEmpty()){

            int[] searchCoords = searchQueue.remove();
            laserID = searchCoords[0];
            azimuthID = searchCoords[1];

            if(laserID - 1 >= 0){
                if(java.lang.Math.abs(distances[laserID - 1][azimuthID][1]) > _heightTolarence){
                    foundPoints++;
                    azimuth = (int) distances[laserID - 1][azimuthID][0];
                    candidate.updateBounds(azimuth, laserID - 1, _frame.getDistance(laserID - 1, azimuth));
                    distances[laserID - 1][azimuthID][1] = 0.0;
                    searchQueue.add(new int[] {laserID - 1, azimuthID});
                }
            }
            if(laserID + 1 < HDL_LASER_PER_FIRING / 2){
                if(java.lang.Math.abs(distances[laserID + 1][azimuthID][1]) > _heightTolarence){
                    foundPoints++;
                    azimuth = (int) distances[laserID + 1][azimuthID][0];
                    candidate.updateBounds(azimuth, laserID + 1, _frame.getDistance(laserID + 1, azimuth));
                    distances[laserID + 1][azimuthID][1] = 0.0;
                    searchQueue.add(new int[] {laserID + 1, azimuthID});
                }
            }
            if(azimuthID - 1 >= 0){
                if(java.lang.Math.abs(distances[laserID][azimuthID - 1][1]) > _heightTolarence){
                    foundPoints++;
                    azimuth = (int) distances[laserID][azimuthID - 1][0];
                    candidate.updateBounds(azimuth, laserID, _frame.getDistance(laserID, azimuth));
                    distances[laserID][azimuthID - 1][1] = 0.0;
                    searchQueue.add(new int[] {laserID, azimuthID - 1});
                }
            }
            if(azimuthID + 1 < number_of_azimuths){
                if(java.lang.Math.abs(distances[laserID][azimuthID + 1][1]) > _heightTolarence){
                    foundPoints++;
                    azimuth = (int) distances[laserID][azimuthID + 1][0];
                    candidate.updateBounds(azimuth, laserID, _frame.getDistance(laserID, azimuth));
                    distances[laserID][azimuthID + 1][1] = 0.0;
                    searchQueue.add(new int[] {laserID, azimuthID + 1});
                }
            }    
        }
        if(foundPoints > _positiveHitsThreshold){
            return candidate;
        }else{
            return null;
        }
    }
    /**
     * Get the number of Obstacles found
     * 
     * @return number of obstacles found
     */
    public int getNumberOfObticles(){
        return _foundObstacles.size();
    }
    /**
     * Clear buffers
     */
    public void clearObsticlesSeen(){
        _foundObstacles.clear();
    }
    /**
     * Get the last Obstacle found and remove from buffer
     * 
     * @return Obstacle found from last analysis ran
     */
    public Obstacle getLatestObstacleFound(){
        if(_foundObstacles.size() == 0){
            return null;
        }
        Obstacle temp = _foundObstacles.get(_foundObstacles.size() - 1);
        _foundObstacles.remove(_foundObstacles.size() - 1);
        return temp;
    }

}
