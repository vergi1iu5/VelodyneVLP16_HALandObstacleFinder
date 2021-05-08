// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package TerrainAnalysis;

import Hardware.VelodyneLidarHDL.PacketDecoder.HDLFrame;

import java.util.ArrayList;
import java.util.Queue;
import java.util.LinkedList;
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

    private static double _heightTolarence; /**<How much does a point's Z-value have differe from _groundRef to trigger a search  */
    private static double _groundRef;       /**<Ground reference */
    private static int _positiveHitsThreshold; /**<How many possitive hits are needed to count a search as valid */
    private ArrayList<Obstacle> _foundObstacles = new ArrayList<Obstacle>(); /**<ArrayList to hold all found Obstacles */

    private final int HDL_LASER_PER_FIRING = 32;
    private final int z_cord = 2;
    
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
        //Get all distances in a 2D array for searching
        frame.getSortedDistances(distances, number_of_azimuths);
        //Begin search (Azimuth/X-axis fast).
        for(int laserID = 0; laserID < HDL_LASER_PER_FIRING / 2; laserID++){
            for(int azimuthID = 0; azimuthID < number_of_azimuths; azimuthID++){
                //If point exceeds threshold, trigger BFS around found point.
                if((java.lang.Math.abs(distances[laserID][azimuthID][1]) > _heightTolarence)){
                    Obstacle obstacle;
                    //If Obstacle found, add to ArrayList
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
        //Get all point-cloud data as a 3D array
        frame.getSortedPointCloud(pointCloud, number_of_azimuths);
        //Begin search (Azimuth/X-Axis fast)
        for(int laserID = 0; laserID < HDL_LASER_PER_FIRING / 2; laserID++){
            for(int azimuthID = 0; azimuthID < number_of_azimuths; azimuthID++){
                double zValue = pointCloud[laserID][azimuthID][z_cord];
                //If point's Z-value exceeds treshold, trigger a BFS around the found point
                if((java.lang.Math.abs(zValue - _groundRef) > _heightTolarence)){
                    Obstacle obstacle;
                    //Add Obstacle to ArrayList if found
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
        //Initialize BFS queue and tracking variables
        Queue<int[]> searchQueue = new LinkedList<>();
        int foundPoints = 1;
        Obstacle candidate;
        double height = pointCloud[laserID][azimuthID][z_cord];
        //Determine if Obstacle is a Creater or a Boulder
        if(height < 0){
            candidate = new CreaterObstacle(pointCloud[laserID][azimuthID], _groundRef);
        }else{
            candidate = new BoulderObstacle(pointCloud[laserID][azimuthID], _groundRef);
        }

        searchQueue.add(new int[] {laserID, azimuthID});
        //Perform BFS
        while(!searchQueue.isEmpty()){
            //Search moves along LaserID and AzimuthID axis and updates bound to outside a possitive hit
            //To ensure entire Obstacle is sourrounded by boudary box
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
        //Return candidate Obstacle if enough points were found
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
        //Height in this context means the difference in distance between the found point
        //and a flat sourface. If coming from a calibrated frame, the distance = 0 if close to a flat plane
        double height = distances[laserID][azimuthID][1];
        int azimuth = (int) distances[laserID][azimuthID][0];
        //If height is < 0, then distance was shorted than a flat sourface inidicating possible Boulder
        if(height < 0){
            candidate = new BoulderObstacle((int) distances[laserID][azimuthID][0], laserID,(double) _frame.getDistance(laserID, azimuth), _groundRef) ;
        }else{
            candidate = new CreaterObstacle((int) distances[laserID][azimuthID][0], laserID,(double) _frame.getDistance(laserID, azimuth), _groundRef);
        }

        searchQueue.add(new int[] {laserID, azimuthID});
        //Start BFS
        while(!searchQueue.isEmpty()){

            int[] searchCoords = searchQueue.remove();
            laserID = searchCoords[0];
            azimuthID = searchCoords[1];
            //Same as fr cartician but now bounds are updated in polar coordinates.
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
        //Return candidate Obstacle if enough points were found
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
