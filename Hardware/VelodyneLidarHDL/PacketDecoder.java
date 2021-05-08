// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package com.AURMC.lib.Hardware.VelodyneLidarHDL;

import java.util.Deque;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.nio.ByteBuffer;
import java.lang.String;
import java.util.Comparator;
/**
 * An interface to define all constants used in the VelodyneLidar package
 */
interface Constants{

    static final int HDL_NUM_BYTES_PER_RETURN = 3;      /**<Number of bytes used to represent a single laser return */
    static final int HDL_NUM_BYTES_PER_BLOCK = 100;     /**<Number of bytes within a single block (one block per azimuth sampled) */
    static final int HDL_NUM_ROT_ANGLES = 36001;        /**<Number of possible azimuth values (360 degrees with resolution of 0.01) */
    static final int HDL_LASER_PER_FIRING = 32;         /**<Number of lasers fired per firing sequence/ azimuth sampled (16 per column, fire twice) */
    static final int HDL_MAX_NUM_LASERS = 64;           /**<Maximum number of laser a Velodyne Lidar can have */
    static final int HDL_FIRING_PER_PKT = 12;           /**<Number of firing sequences/ azimuths sampled contained in each packet */
    static final int PORT_NUMBER = 2368;                /**<Default port number */
    static final int BLOCK_START_FLAG = 0xFFEE;         /**<Flag to indicate start of new block within packet */
    static final int PACKET_HEADER_OFFSET = 0x0000;     /**<Flag to indicate end of header inside GPS packet */
    static final int PACKET_HEADER_SIZE = 42;           /**<Number of bytes inside GPS packet used for header */
    static final int PACKET_UNUSED_OFFSET = 0x002A;     /**<Flag to indicate end of unused bytes within GPS packet */
    static final int PACKET_UNUSED_SIZE = 198;          /**<Number of unused bytes within GPS packet */
    static final int PACKET_TIMESTAMP_OFFSET = 0x00F0;  /**<Flag to indicate start of GPS timestamp */
    static final int PACKET_TIMESTAMP_SIZE = 4;         /**<Number of bytes used to indicate timestamp */
    static final int LIDAR_ROTATION_ANGLE = 9000;        /**<Lidar's physical rotation angle. Positive angle indicates CCW rotation */
    static final int LIDAR_TILT_ANGLE = 3760;           /**<Lidar inclination angle * 100. Possitive value means tilted down. */
    static final double LIDAR_HIGHT_M = 0.383;          /**<Lidar's height in meters */
    static final double LIDAR_RESOLUTION_M = 0.002;     /**<Lidar's resolution in meters */
    /**
     * Enum used to give cordinate-based indexing
     */
    public static enum pointCouldCoord{
        x, y, z;
    }
}
/**
 * PacketDecoder class
 *
 * <p>The PacketDecoder class is intended to be used alonside the PacketDriver inside the VelodyneLidar wrapper class
 *
 * <p>DecodePacket(Byte[] packet) function is intended to be used to convert packet into a frame (explained in HDLFrame class)
 * <p>GetLatestFrame() function is intended to be used to retreive the lastest HDLFrame with all of the provided decoded packets
 */
public class PacketDecoder {
    
    private boolean _generatePointCloud; /**<Indicates if point cloud data is meant to be generated (saves RAM if not) */
    public static double[] Lidar_height_map = new double[Constants.HDL_NUM_ROT_ANGLES]; /**<Array to map an specific azimuth angle to the exact height of the sensors at that azimuth */
    public static double[] Az_cos_lookup_table = new double[Constants.HDL_NUM_ROT_ANGLES]; /**<Azimuth angle cosine lookup table to speed up computation */
    public static double[] Az_sin_lookup_table = new double[Constants.HDL_NUM_ROT_ANGLES]; /**<Azimuth angle sine lookup table to speed up coputation */
    public static double[][] El_cos_lookup_table = new double[Constants.HDL_LASER_PER_FIRING][Constants.HDL_NUM_ROT_ANGLES]; /**<Elevation angle cosine lookup table */
    public static double[][] El_sin_lookup_table = new double[Constants.HDL_LASER_PER_FIRING][Constants.HDL_NUM_ROT_ANGLES]; /**<Elevation angle sine lookup table */
    public static int[]    elAngle_lookup_table = {1500, -100, 1300, -300, 1100, -500, 900, -700,   /**<Table to map laser id to elevation angle */
                                                    700, -900, 500, -1100, 300, -1300, 100, -1500};
    public static int[] laserIdMap = {15,13,11,9,7,5,3,1,14,12,10,8,6,4,2,0}; /**<Table to map firing sequence to laserID (i.e laser id 15 gets fired first) */                                            
    /**
     * HDLLaserReturn class implements Serializable to assure continous data representation
     * 
     * <p>The HDLLaserReturn class is meant to be the bulding blocks for the HDLFiringData class
     * 
     * <p>getDistance() function returns the raw data distance as a zero-padded integer
     * <p>getIntensity() function returns the raw data intensity as a zero-padded integer
     */
    public class HDLLaserReturn implements java.io.Serializable{
    
        private byte[] _distance = new byte[2];
        private byte _intensity;
        private static final long serialVersionUID = 0;
        /**
         * Construtor for creating a HDLLaserReturn object from bytes directly out of packet
         * @param rawData Byte array containing the three bytes that make up a single laser return sample
         */
        public HDLLaserReturn(byte[] rawData){
            this._distance[0] = rawData[0]; //Distance LSB
            this._distance[1] = rawData[1]; //Distance MSB
            this._intensity   = rawData[2];
        }
        /**
         * Constructor to create a HDLLaserReturn object from distance and intesity values
         * @param _distance Byte array containing MSB and LSB distance values
         * @param _intensity Single byte to represent the intensity of the return
         */
        public HDLLaserReturn(byte[] _distance, byte _intensity){
            this._distance = _distance.clone();
            this._intensity = _intensity;
        }
        /**
         * Get distance as an integer made up of the two distance bytes
         * @return Integer value for distance (raw value, need to multiply by resolution)
         */
        public int getDistance(){
            byte[] temp = {0x00, 0x00, _distance[1], _distance[0]};
            ByteBuffer wrapped = ByteBuffer.wrap(temp);
            return wrapped.getInt();
        }
        /**
         * Get intensity as a byte
         * @return intesity represented as an unsigned char
         */
        public short getIntesity(){
            byte[] temp = {0x00, _intensity};
            ByteBuffer wrapped = ByteBuffer.wrap(temp);
            return wrapped.getShort();
        }
    }
    /**
     * HDLFiringData class implements Serializable to assure data continuity. It is made up of HDLFiringReturns and 
     * serves as the building block for the HDLFirigPacket class.
     * 
     * <p>getAzimuthAngle() function returns the azimuth angle at which the firing group was fired to produce the firing block
     * <p>getLaserReturn(laserID) function returns the HDLLaserReturn object for the provided laser id
     * <p>addLaserReturn(laserReturn) adds a single HDLLaserReturn object to the arrayList of laser returns 
     */
    public class HDLFiringData implements java.io.Serializable{
    
        private byte[] _blockIdentifier = new byte[2];
        private byte[] _azimuthAngle    = new byte[2]; /**<Firing group azimuth angle in degrees */
        private ArrayList<HDLLaserReturn> _laserReturns = new ArrayList<HDLLaserReturn>(); /**<ArrayList containing all 32 laser returns per block */

        private static final long serialVersionUID = 1;
        /**
         * Constructor for creating a HDLFiringData object from an array of bytes (usually a sub-array of packet)
         * @param rawData byte array containing all data needed to generate block
         * @param data_length size of provided data array
         */
        public HDLFiringData(byte[] rawData, int data_length){
            //First four bytes are used for block identifier and azimuth angle
            this._blockIdentifier[0] = rawData[0];
            this._blockIdentifier[1] = rawData[1];
            this._azimuthAngle[0] = rawData[2];
            this._azimuthAngle[1] = rawData[3];
            //Break down raw data array into each individual HDLLaserReturn object
            for(int i = 0; i < Constants.HDL_LASER_PER_FIRING; i++){
                int offset = (i*Constants.HDL_NUM_BYTES_PER_RETURN + 4);
                byte[] laserReturnRawData = Arrays.copyOfRange(rawData, offset, offset + Constants.HDL_NUM_BYTES_PER_RETURN);
                this._laserReturns.add(new HDLLaserReturn(laserReturnRawData));
            }
        }
        /**
         * Constructor for creaeting HDLFiringData object without any HDLLaserReturn objects within object.
         * Function addLaserReturn must then be used to populate array
         * @param blockIdentifier byte array containing the two bytes used to identify a new block
         * @param azimuthAngle byte array containing the two bytes used to represent the azimuth angle (MSB first)
         */
        public HDLFiringData(byte[] blockIdentifier, byte[] azimuthAngle){
            this._blockIdentifier = blockIdentifier.clone();
            this._azimuthAngle = azimuthAngle.clone();
        }
        /**
         * Constructor for creating a HDLFringData object with all of the HDLLaserReturn object already instantiated
         * @param blockIdentifier byte array containing the two bytes used to identify a new block
         * @param azimuthAngle byte array containing the two bytes used to represent the azimuth angle (MSB first)
         * @param laserReturns ArrayList containing all of the 32 laserReturn objects needed to make a full block
         */
        public HDLFiringData(byte[] blockIdentifier, byte[] azimuthAngle, ArrayList<HDLLaserReturn> laserReturns){
            this._blockIdentifier = blockIdentifier.clone();
            this._azimuthAngle = azimuthAngle.clone();
            this._laserReturns.addAll(laserReturns);
        }
        /**
         * Function to get block identifier as an unsigned char
         * @return block indentifier as an unsinged char
         */
        public short getBlockIdentifier(){
            ByteBuffer wrapped = ByteBuffer.wrap(_blockIdentifier);
            return wrapped.getShort();
        }
        /**
         * Function to get azimuth angle at which the firing group was fired to create block
         * @return Azimuth angle as an integer
         */
        public int getAzimuthAngle(){
            byte[] temp = {0x00, 0x00, _azimuthAngle[1], _azimuthAngle[0]};
            ByteBuffer wrapped = ByteBuffer.wrap(temp);
            return wrapped.getInt();
        }
        /**
         * Get a single laser return within the firing block
         * @param laserID indetifier for requested laser return
         * @return single laser return as an HDLLaserReturn object
         */
        public HDLLaserReturn getLaserReturn(int laserID){
            if(laserID < Constants.HDL_LASER_PER_FIRING && laserID < this._laserReturns.size()){
                return _laserReturns.get(laserID);
            }else{
                return _laserReturns.get(Constants.HDL_LASER_PER_FIRING - 1);
            } 
        }
        /**
         * Add a single HDLLaseReturn to ArrayList if it contains less than HDL_LASER_PER_FIRING
         * @param laserReturn single HDLLaseReturn to be added
         */
        public void addLaserReturn(HDLLaserReturn laserReturn){
            if(this._laserReturns.size() < Constants.HDL_LASER_PER_FIRING){
                this._laserReturns.add(laserReturn);
            }
        }
    }
    /**
     * HDLDataPacket implements Serializable to ensure all data is continous. It is made up of HDLFiringData objects
     * and serves to reprent a full packet broken down into its building blocks.
     * 
     * <p>getTimeStamp() function returns the time at which the packet was created
     * <p>getFiringData(blockId, firingData[out]) function returns the firing data for the indentifier provided.
     * <p>getFirinfData(blockId) function returns the firind data, as a HDLFiringData object, for th indentifier provided.
     * <p>addFiringData(HDLFiringData) function adds the already instantiated HDLFiring object to the ArrayList 
     */
    public class HDLDataPacket implements java.io.Serializable{
    
        private ArrayList<HDLFiringData> _firingData = new ArrayList<HDLFiringData>(); /**<ArrayList used to keep all blocks for this Packet object */
        private byte[] _gpsTimestamp = new byte[4]; /**<Timestamp for sampled data. Not used for for added for completion sake */
    
        private static final long serialVersionUID = 2;
        /**
         * Construtor for creating an HDLDataPacket from a raw packet directly extracted from lidar socket
         * 
         * @param rawData byte array containing entire packet from socket
         * @param data_length number of bytes contained in array
         */
        public HDLDataPacket(byte[] rawData, int data_length){
            //Throw warning if array is not expected size.
            if(data_length != 1206){
                System.out.println("PacketDecoder: Warning! Packet size is not 1206 bytes long.");
            }
            //Break down packet into blocks which will be further broken down into single returns
            for(int i = 0; i < Constants.HDL_FIRING_PER_PKT; i++){
                int offset = i * Constants.HDL_NUM_BYTES_PER_BLOCK;
                byte[] blockRawData = Arrays.copyOfRange(rawData, offset, offset + Constants.HDL_NUM_BYTES_PER_BLOCK);
                this._firingData.add(new HDLFiringData(blockRawData, Constants.HDL_NUM_BYTES_PER_BLOCK));
            }
            //Get timestamp
            this._gpsTimestamp = Arrays.copyOfRange(rawData, data_length - 6, data_length - 2);
        }
        /**
         * Get timestamp as a zero-padded integer
         * @return timestamp integer
         */
        public int getTimestamp(){
            ByteBuffer wrapped = ByteBuffer.wrap(_gpsTimestamp);
            return wrapped.getInt();
        }
        /**
         * Get firing data for an specific block (0 - 11)
         * @param blockID integer specifying which block within packet to get
         * @param firingData return value
         */
        public void getFiringData(int blockID, HDLFiringData[] firingData){
            if(blockID < _firingData.size()){
                firingData[0] = _firingData.get(blockID);
            }else{
                firingData[0] = _firingData.get(_firingData.size() - 1);
            }
        }
        /**
         * Get firing data for an specific block (0 - 11). Returns as an actual HDLFiringData object which makes it slower
         * @param blockID Integer specifying which block within packet to get
         * @return HDLFiringData object requested
         */
        public HDLFiringData getFiringData(int blockID){
            if(blockID < _firingData.size()){
                return _firingData.get(blockID);
            }else{
                return _firingData.get(_firingData.size() - 1);
            }
        }
        /**
         * Add a single firingData object to ArrayList if it contains less than HDL_FIRING_PER_PKT entries
         * @param firingData HDLFiringData to be added
         * @return Boolean value indicating if block was added or not
         */
        public boolean addFiringData(HDLFiringData firingData){
            if(_firingData.size() < Constants.HDL_FIRING_PER_PKT){
                _firingData.add(firingData);
                return true;
            }else{
                return false;
            }   
        }
    } 
    /**
     * HDLFrame class intended to serve as an "snapshot" of what the lidar sees given the packets added to the frame. The entries are sorted
     * in a table-like fashion with the laserID and azimuth angle serving as the vertical and horizontal axis respectivetly. For the point cloud
     * data, it is a 3D table with the cartician cordinate as the third axis (order: X,Y,Z).
     * 
     * <p>addpoint() function is called to add a single point cloud entry into the frame. The laser ID and azimuth angle are used to index point.
     * <p>addDistance() function is called to add a single distance sample into the frame. The laser ID and azimuth anfle are used to index entry.
     * <p>getPoint() function is used to retrieve a single point cloud point given a laser ID and azimuth angle.
     * <p>getDistance() function is used to retrieve a single distance entry given a laser ID and azimuth angle.
     * <p>getNumberOfAzimuthsInFrame() function is used to determine how many unique firing sequences (one per azimuth) were used to create frame.
     * <p>getRowForLaserID() function is used to get all point cloud entries that correspond to a single laser ID.
     * <P>getDistanceRowForLaserID() function is used to get all distance entries that correspond to a single laser ID.
     * <p>getSortedPointCloud() function returns the entire point cloud 3D array which comes sorted based on the cartician values.
     */
    public class HDLFrame{

        private HashMap<Integer,double[][]> _pointCould3DArray = new HashMap<Integer,double[][]>(); /**<HashMap to store 3D point cloud data. Azimuth angle serves as the key */
        private HashMap<Integer,double[]> _distance2DArray     = new HashMap<Integer,double[]>();   /**<HashMap to store 2D distance data. Azimuth angle serves as the key */
        private int number_of_azimuths = 0; /**<Total number of azimuths used to create the frame */
        /**
         * Default constructor. Use the other functions to populate.
         */
        public HDLFrame(){
            
        }
        /**
         * Add a single point cloud entrie into frame.
         * 
         * @param point point to be added. It should be a double array contaning X, Y, and Z values.
         * @param laserID integer representing the laser which was used to create the point
         * @param azimuth azimuth angle, as integer, for point
         */
        public void addPoint(double[] point, int laserID, int azimuth){
            //If both the laser ID and angle are within limits
            if(laserID < Constants.HDL_LASER_PER_FIRING && azimuth < Constants.HDL_NUM_ROT_ANGLES){
                //Create new entry in HashMap if there is none for azimuth
                if(!(_pointCould3DArray.containsKey(azimuth))){
                    double[][] temp = new double[Constants.HDL_LASER_PER_FIRING / 2][3];
                    _pointCould3DArray.put(azimuth, temp);
                    number_of_azimuths++;
                }
                //Get entry corresponding to key (azimuth), add point, then add back to HashMap
                double[][] temp = _pointCould3DArray.get(azimuth);
                for(int i = 0; i < point.length && i < 3; i++){
                    temp[laserID][i] = point[i];
                }
                _pointCould3DArray.put(azimuth, temp);
            }
        }
        /**
         * Add a single distance entry to frame.
         * 
         * @param dist Double value indicating distance in meters
         * @param laserID Integer representing the laser from which the distance was measured
         * @param azimuth azimuth angle, as integer, for which the distance measurement was taken
         */
        public void addDistance(double dist, int laserID, int azimuth){
            //Add distance entry if angle is valid (0 - 36000)
            if( azimuth < Constants.HDL_NUM_ROT_ANGLES){
                //Create new entry in HashMap if there is none for azimuth
                if(!(_distance2DArray.containsKey(azimuth))){
                    double[] temp = new double[Constants.HDL_LASER_PER_FIRING / 2];
                    _distance2DArray.put(azimuth, temp);
                }
                //Add distance to array using laserId as index
                double[] temp = _distance2DArray.get(azimuth);
                temp[laserID] = dist;
                _distance2DArray.put(azimuth, temp);
            }
        }
        /**
         * Get a single point that corresponds to a single laser Id and azimuth.
         * 
         * @param laserID ID number for which to get the point for
         * @param azimuth Azimuth angle for which to get point for
         * @return Point cloud entry if found, NULL if not found.
         */
        public double[] getPoint(int laserID, int azimuth){
            if(_pointCould3DArray.containsKey(azimuth)){
                return _pointCould3DArray.get(azimuth)[laserID];
            }else{
                return null; 
            }
        }
        /**
         * Get a single distance entry that corresponds to a single laser id and azimuth.
         * 
         * @param laserID laser ID number for which to get the point for
         * @param azimuth Azimuth angle for which to get point for
         * @return Distance as double. Returns NULL if no entry found
         */
        public double getDistance(int laserID, int azimuth){
            if(_distance2DArray.containsKey(azimuth)){
                return _distance2DArray.get(azimuth)[laserID];
            }else{
                return this._interpolateDistance(_distance2DArray, azimuth, laserID);
            }
        }
        /**
         * Calculate predicted Distance if no entry is found. Uses the two closes azimuths angle to
         * perform linear interpolation. Works better the more data points the frame contains.
         * 
         * @param map HashMap containing all distanance to be used in interpolation
         * @param az Azimuth angle which was not found in map so it required interpolation
         * @param ID Laser Id for which distance was requested
         * @return Interpolated distance as double
         */
        private double _interpolateDistance(HashMap<Integer,double[]> map, int az, int ID){
            //Move lowerBound and upperBound as close as possible to az. Must be valid entries in map.
            int lowerBound = 0;
            int upperBound = 36000;
            for(int azimuth : map.keySet()){
                 if(azimuth < az && azimuth > lowerBound){
                     lowerBound = azimuth;
                 }
                 if(azimuth > az && azimuth < upperBound){
                     upperBound = azimuth;
                 }
            }
            //Perform linear interpolation with the found bounds.
            double low = map.get(lowerBound)[ID];
            double high = map.get(upperBound)[ID];
            double slope = (low - high)/(lowerBound - upperBound);
            double change = slope*(az - lowerBound);
            return low + change;
        }
        /**
         * Get the number of azimuths that currently make up the frame
         * 
         * @return number of azimuths as an integer
         */
        public int getNumberOfAzimuthsInFrame(){
            return number_of_azimuths;
        }
        /**
         * Get all points within frame for a single laser ID. Points return sorted in the X and Y directions.
         * 
         * @param laserID Laser for which user wishes to obtain all points within frame
         * @param return_data data buffer which will contain all sorted points once function returns
         * @param data_length Number of azimuths in the returned data buffer
         */
        public void getRowForLaserID(int laserID, double[][] return_data, int data_length){
            //Make sure requeted data size matches that contained in frame
            if(data_length != number_of_azimuths){
                return;
            }
            
            int index = 0;
            //Add all azimuths to return_data buffer and sort based on X axis
            for(int azimuth : _pointCould3DArray.keySet()){
                for(int i = 0; i < 3; i++){
                    return_data[index][i] = _pointCould3DArray.get(azimuth)[laserID][i];
                }
                index++;
            }
            Arrays.sort(return_data, new Comparator<double[]>(){
               @Override
               public int compare(double[] o1, double[] o2){
                  return Double.compare(o1[0], o2[0]);
               }
            });
        }
        /**
         * Get distance entries within frame for a single laser ID. Returned data is sorted based on the azimuth angle.
         * 
         * @param laserID Laser for which user wishes to obtain all points within frame
         * @param return_data data buffer which will contain all sorted points once function returns
         * @param data_length Number of azimuths in the returned data buffer
         */
        public void getDistanceRowForLaserID(int laserID, double[][] return_data, int data_length){
            //Make sure return data buffer size matches number of entries in frame
            if(data_length != number_of_azimuths){
                return;
            }
            
            int index = 0;
            //Add all distances to return_data and sort based on azimuth angle
            for(int azimuth : _pointCould3DArray.keySet()){
                return_data[index][0] = azimuth;
                if(!_generatePointCloud){
                    double calibrationDistance = _calibrationFrame.getDistance(laserID, azimuth);
                    return_data[index][1] = _distance2DArray.get(azimuth)[laserID] - calibrationDistance;
                }
                index++;
            }
            Arrays.sort(return_data, new Comparator<double[]>(){
               @Override
               public int compare(double[] o1, double[] o2){
                  double az1, az2;
                  //Take signed modulus of azimuth angle based on Lidar's rotation angle 
                  //i.e. if az = 345 and lidar is rotated by 90 deg CW, then angle should be 65 deg
                  if(o1[0] > Constants.LIDAR_ROTATION_ANGLE){
                     az1 = o1[0] - 36000;
                  }else{
                     az1 = o1[0]; 
                  }
                  if(o2[0] > Constants.LIDAR_ROTATION_ANGLE){
                     az2 = o2[0] - 36000;
                  }else{
                     az2 = o2[0]; 
                  }
                  return Double.compare(az1, az2);
               }
            });
        }
        /**
         * Get all distances as a 2D array and sort them based on azimuth angle.
         * 
         * @param distances Array which will contain all returned data.
         * @param number_of_azimuths Size of distances
         */
        public void getSortedDistances(double[][][] distances, int number_of_azimuths){
            //Make sure return_data will fit all entries in frame
            if(this.number_of_azimuths != number_of_azimuths){
                System.out.println("HDLFrame: Warning - input 3D array does not match frame's avalible points.");
                return;
            }
            //Get all entries
            for(int laserID = 0; laserID < Constants.HDL_LASER_PER_FIRING / 2; laserID++){
                getDistanceRowForLaserID(laserIdMap[laserID], distances[laserID], number_of_azimuths);
            }
        }
        /**
         * Get point cloud data as a 3D array and sorted on both X and Y directions.
         * 
         * @param pointCloud    Return data
         * @param number_of_azimuths    Size of pointCloud
         */
        public void getSortedPointCloud(double[][][] pointCloud, int number_of_azimuths){
            //Make sure return data can fit all entries in frame
            if(this.number_of_azimuths != number_of_azimuths){
                System.out.println("HDLFrame: Warning - input 3D array does not match frame's avalible points.");
                return;
            }
            //Get all points found in frame
            for(int laserID = 0; laserID < Constants.HDL_LASER_PER_FIRING / 2; laserID++){
                getRowForLaserID(laserIdMap[laserID], pointCloud[laserID], number_of_azimuths);
            }
        }
    }

    //private String _correction_file;
    private int _last_azimuth; /**<Last azimuth that was pushed to a frame */
    private int _max_num_of_frames; /**<Maximum number of frames to be stores in decoder at once */
    private HDLFrame _frame; /**<Current fram to which data is being added to */
    private HDLFrame _calibrationFrame; /**<Frame used to what a flat surface should look like to the lidar */
    private Deque<HDLFrame> _frames = new LinkedList<HDLFrame>(); /**<Deque to store all frames created. Can Fit up to _max_num_of_frames */
    /**
     * Constructor to PacketDecoder class. Input is used to indicate if the algorithm should take the 
     * packet and derive a point cloud 3D array or just store as distances (i.e. polar coordinates). Point
     * cloud is only needed if no calibration data is avalible (flat surface was not sampled).
     * 
     * @param generatePointCloud Flag to allow point cloud calculations.
     */
    public PacketDecoder(boolean generatePointCloud){
        this._generatePointCloud = generatePointCloud;
        this._max_num_of_frames = 3;
        UnloadData();
        InitTables();
        System.out.println("PacketDecoder: Succesfully initialized decoder.");
    }

    @Override
    public void finalize(){

    }
    /**
     * Set the number of frames allowed to be stored at once.
     * 
     * @param max_num_frames Interger indicating number of frames to be stored at once in Queue.
     */
    public void SetMaxNumberOffFrames(int max_num_frames){
        if(max_num_frames <= 0){
            return;
        }else{
            _max_num_of_frames = max_num_frames;
        }
        //Remove frames until size met
        while(_frames.size() >= _max_num_of_frames){
            _frames.removeFirst();
        }
    }
    /**
     * Decode a single packet and add to current frame.
     * 
     * @param data raw byte array containing a packet coming straight from lidar
     * @param data_length number of bytes within array
     */
    public void DecodePacket(byte[] data, int[] data_length){
        //Make sure array is of the expected size
        if(data_length[0] != 1206){
            System.out.println("PacketDecoder: warning! data packet has abnormal size");
            return;
        }
        //Create packet and add to frame
        ProcessesHDLPacket(data, data_length[0]);
    }
    /**
     * Protected function to decode a single packet and add to current frame. Function can only be called once
     * it is determined packets is valid.
     * 
     * @param data  Raw byte array containing a packet coming straight from lidar
     * @param data_length   Number of bytes within array
     */
    protected void ProcessesHDLPacket(byte[] data, int data_length){
        //Create new HDLDataPacket with the raw data provided
        HDLDataPacket dataPacket = new HDLDataPacket(data, data_length);
        //Processes all blocks within HDLDataPacket
        for(int BlockID = 0; BlockID < Constants.HDL_FIRING_PER_PKT; ++BlockID){
            
            HDLFiringData firingData = dataPacket.getFiringData(BlockID);
            /**
             * Uncomment this code out if you wish to split frames once the lidar loops back arround
             *  if(firingData.getAzimuthAngle() < _last_azimuth){
             *      splitFrame();
             *  }
             * 
             * _last_azimuth = firingData.getAzimuthAngle();
             */
            //Processes all laser returns within each block
            for(int laserID = 0; laserID < Constants.HDL_LASER_PER_FIRING; laserID++){
                //Add slight azimuth drift if return comes from second firing sequence (all lasers fired twice for a single azimuth angle)
                int az = (laserID >= 16)? (firingData.getAzimuthAngle() + 10) % 36000 : firingData.getAzimuthAngle();
                //Perform signed modulus 36000 based on lidar's physical rotation angle
                int temp = Constants.LIDAR_ROTATION_ANGLE - az;
                int azimuth = (temp < 0) ? (36000 - (java.lang.Math.abs(temp) % 36000) ) % 36000: (temp % 36000);
                PushFringData(laserID % 16, azimuth, firingData.getLaserReturn(laserID), false);
            }
        }
    }
    /**
     * Push firing data to current frame or calibration frame. Generates point clod data if decoder configured to do so.
     * 
     * @param laserID Laser identifier for sensor used to sample laser return
     * @param azimuth Azimuth at which return was taken
     * @param laserReturn HDLLaserReturn containing distance and intensity data
     * @param isCalibrationData Indicates if the data is for a flat plane and should be added to clalibration frame
     */
    protected void PushFringData(int laserID, int azimuth, HDLLaserReturn laserReturn, boolean isCalibrationData){
        //Get all cosines and sines needed to perform point cloud calculations
        double cosAzimuth = Az_cos_lookup_table[azimuth];
        double sinAzimuth = Az_sin_lookup_table[azimuth];
        double cosElevation = El_cos_lookup_table[laserID][0];
        double sinElevation = El_sin_lookup_table[laserID][0];
        //Get distance in meters based on lidar's resolution
        double distance_meters = laserReturn.getDistance() * Constants.LIDAR_RESOLUTION_M;
        double X = 0.0, Y = 0.0, Z = 0.0, xy_plane_projection = 0.0;
        //Perform point cloud calculations if decoder configured to do so
        if(_generatePointCloud){
            xy_plane_projection = distance_meters * cosElevation;
            X = xy_plane_projection * sinAzimuth;
            Y = xy_plane_projection * cosAzimuth;
            Z = distance_meters * sinElevation;
        }
        //Add data to regular frame if not comming from flat plane
        if(!isCalibrationData){
            //If avalible, use calibration frame to determine lidar's height
            if(_calibrationFrame.getPoint(laserID, azimuth) != null){
                Z += -(_calibrationFrame.getPoint(laserID, azimuth)[2]);
            }else{//If not avalible, use lidar height to estimate lidar's height at the current azimuth.
                Z += Lidar_height_map[azimuth];
            }
        }

        double[] point = {X,Y,Z};
        //Add to calibration frame if data is for a flat plane.
        if(isCalibrationData){
            _calibrationFrame.addPoint(point, laserID, azimuth);
            _calibrationFrame.addDistance(distance_meters, laserID, azimuth);
        }else{//Add to regular frame if not
            _frame.addPoint(point, laserID, azimuth);
            _frame.addDistance(distance_meters, laserID, azimuth);
        }
    }
    /**
     * Function used to push a raw packet into the calibration frame. Ususally used right after a flat plane
     * has been sampled.
     * 
     * @param data Raw data packet coming staright from lidar socket
     * @param data_length Size of data buffer
     */
    public void addToCalibrationFrame(byte[] data, int[] data_length){
        //Create an HDLDataPacket object from provided packet
        HDLDataPacket dataPacket = new HDLDataPacket(data, data_length[0]);
        //Processes all blocks within data packet
        for(int BlockID = 0; BlockID < Constants.HDL_FIRING_PER_PKT; ++BlockID){
            
            HDLFiringData firingData = dataPacket.getFiringData(BlockID);
            //Processes all laser returns within block
            for(int laserID = 0; laserID < Constants.HDL_LASER_PER_FIRING; laserID++){
                int az = (laserID >= 16)? (firingData.getAzimuthAngle() + 10) % 36000 : firingData.getAzimuthAngle();
                int temp = 9000 - az;
                int azimuth = (temp < 0) ? (36000 - (java.lang.Math.abs(temp) % 36000) ) % 36000: (temp % 36000);
                //Push calibration data to calibration frame
                PushFringData(laserID % 16, azimuth, firingData.getLaserReturn(laserID), true);
            }
        }
    }
    /**
     * Function used to load factory specified corrections.
     * 
     * @param corrections_file String for name of correction file
     */
    public void SetCorrectionsFile(final String corrections_file){

    }
    /**
     * Get all frames within Queue
     * 
     * @return Queue containing all frames currently in decoder.
     */
    public Deque<HDLFrame> GetFrames(){
        return _frames;
    }
    /**
     * Unload all frames.
     */
    public void ClearFrames(){
        _frames.clear();
    }
    /**
     * Get the lastest frame added to queue.
     * 
     * @param numberOfAzimuthsInFrame Minimum number of azimuths that frame must contain to be retrived.
     * @return HDLFrame requested
     */
    public HDLFrame GetLatestFrame(int numberOfAzimuthsInFrame){
        if(_frame.number_of_azimuths > numberOfAzimuthsInFrame){
            HDLFrame temp = _frame;
            _frame = new HDLFrame();
            return temp;
        }else{
            return null;
        }
        //if(_frames.size() > 0){
        //    HDLFrame temp = _frames.removeLast();
        //    _frames.clear();
        //    if(temp.getNumberOfAzimuthsInFrame() > 300){
        //        return temp;
        //    }else{
        //        return null;
        //    }
        //}else{
        //    return null;
        //}
    }
    /**
     * Clear all variables used to keep track of frames being decoded.
     */
    protected void UnloadData(){
        this._last_azimuth = 0;
        this._frame = new HDLFrame();
        this._frames.clear();
        this._calibrationFrame = new HDLFrame();
    }
    /**
     * Initialize all tables used in point cloud calculations. Lidar height is estimated by
     * taking the point cloud Z values for the two outer most azimuth angles, and linearly 
     * interpolate each one with the center-most azimuth.
     */
    protected void InitTables(){
        //Create azimuth cosine and sin lookup tables.
        //Create lidar_height map estimation.
        for(int i = 0; i < Constants.HDL_NUM_ROT_ANGLES; i++){
            double rad = this.HDL_Grabber_toRadians(i / 100.0);
            Az_cos_lookup_table[i] = java.lang.Math.cos(rad);
            Az_sin_lookup_table[i] = java.lang.Math.sin(rad);
            if(i < 9000){
                Lidar_height_map[i] = (0.00002285714285714286*i + 0.383);
            }else{
               Lidar_height_map[i] = -0.000037142857142857143*(i - 36000) + 0.383;
            }
        }
        //Create elevation lookup tables
        for(int i = 0; i < Constants.HDL_LASER_PER_FIRING / 2; i++){
            for(int az = 0; az < Constants.HDL_NUM_ROT_ANGLES; az++){
               int elAngle = elAngle_lookup_table[i];
               int angleDrift;
               if(az < 9000){
                  angleDrift = (int) (-0.16*az + 3760);
               }else{
                  angleDrift = (int) (0.16*(az - 36000) + 3760);
               }
               elAngle -= angleDrift;
               double rad = this.HDL_Grabber_toRadians(elAngle / 100.0);
               El_cos_lookup_table[i][az] = java.lang.Math.cos(rad);
               El_sin_lookup_table[i][az] = java.lang.Math.sin(rad);
            }
        }
    }

    protected void LoadCorrectionsFile(final String correctionsfile){

    }

    protected void SetCorrectionsCommon(){

    }
    /**
     * Function to create a new frame once lidar wraps arround.
     */
    protected void splitFrame(){
        if(_frames.size() == _max_num_of_frames - 1){
            _frames.removeFirst();
        }
        _frames.addLast(_frame);
        _frame = new HDLFrame();
    }
    /**
     * Convert angle in degrees to radians
     * 
     * @param angle Angle in degrees
     * @return Angle in radians
     */
    private double HDL_Grabber_toRadians(double angle){
        return (angle * java.lang.Math.PI / 180);
    }  
}
