// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package Hardware.VelodyneLidarHDL;

import TerrainAnalysis.Obstacle;
import TerrainAnalysis.ObstacleFinder;

import java.io.File;
import java.io.FileInputStream;
import java.util.Arrays;
/**
 * VelodyneLidar class used to wrap PacketDriver, PacketDecoder, and ObstacleFinder classes. This class serves 
 * as the top-most abstraction layer for using the Velodyne VLP-16 for simple obtacle detection and avoidance.
 * 
 * <p>calibrateLidar() function gets called if there is a file containing the raw packet data of a flat surface.
 * <p>scanFullFieldOfView() function gets called to perform a full FOV scan and produce a single HDLFrame.
 * <p>getClosetObstacle() fucntion is used to used the latest scan, look in it for possible obstacles, and return closest.
 * <p>clearAllDataBuffers() function is used to reset lidar (calibration frame is kept).
 */
public class VelodyneLidar {

    private static PacketDecoder.HDLFrame _latestFrame; /**<Last HDLFrame created from a full FOV scan */
    private static PacketDriver _driver;                /**<PacketDriver used to extract packets from lidar socket */
    private static PacketDecoder _decoder;              /**<PacketDecoder used to produce HDLFrames from packets provided by _driver */
    private static ObstacleFinder _obstacleFinder;      /**<ObstacleFinder used to extract any posible obstacles within _latesFrame */
    private static boolean _isCalibrated = false;       /**<Flag to indicate if a clalibration frame has been fed to _decoder */
    private static boolean _generatePointCloud;         /**<Flag to indicate if _decoder is performing point-cloud calculations */
    private static int _number_azimuths_in_frame;       /**<Number of azimuths required to be sampled before creating and anlysig a HDLFrame */
    /**
     * Main VelodyneLidar class intended to be used intandum with all the other classes in the package. It is the top-most
     * abstraction layer and as such careful consideration must be taken when providing the initializion parameters.
     * 
     * @param heightTolarence   Double to indicate at what height (in meters) to start checking for possible obstacles.
     * @param groundRef Double to idicate (in meters) what the lidar should consider to be ground (i.e. if 0.01 or 0.00 should be ground)
     * @param positiveHitsThreshold Number of laser returns indicating a possible obstacle needed to count as a Obstacle
     * @param numberOfAzimuthsInFrame   Number of azimuths required to be sampled befre creating a HDLFrame
     * @param generatePointCloud    Flag to idicate if point cloud calculations should be performed. If not, then lidar uses polar cordinates to look for obstacles
     */
    public VelodyneLidar(double heightTolarence, double groundRef, int positiveHitsThreshold, int numberOfAzimuthsInFrame, boolean generatePointCloud){
        _driver = new PacketDriver(Constants.PORT_NUMBER);
        _decoder = new PacketDecoder(generatePointCloud);
        _generatePointCloud = generatePointCloud;
        _obstacleFinder = new ObstacleFinder(heightTolarence, groundRef, positiveHitsThreshold);
        _number_azimuths_in_frame = (numberOfAzimuthsInFrame > 350)? numberOfAzimuthsInFrame : 350; 
        scanFullFieldOfView();
    }
    /**
     * Funtion used to load calibration file and feed it to _decoder to generate the calibraiton frame.
     * Calibration is required if the user does not want to perform point-cloud calculations. Calibrations
     * makes looking for obtacles more accurate in both Cartician and Polar cordinate searches.
     * 
     * @return Flag indicating if Lidar was successfully calibrated or not.
     */
    public boolean calibrateLidar(){
        // IMPORTANT: MAKE SURE TO REPLACE DIRECTORY WITH THE ONE FOR YOUR OWN SETUP
        File file = new File("/home/lvuser/CalibrationData/cal4.txt");
       
        byte[] bytes = readContentIntoByteArray(file);
        //If file not found, then return False
        if(bytes == null){
            return (_isCalibrated = false);
        }
        _isCalibrated = true;

        int[] length = {1206};
        //Push entire calibration file into calibration file
        int num_packets_in_file = 5000;
        // IMPORTANT: MAKE SURE TO SPECIFY HOW MANY PACKETS WERE USED TO CREATE CALIBRATION FILE
        for(int i = 0; i < num_packets_in_file; i++){
            int offset = i * 1206;
            byte[] data = Arrays.copyOfRange(bytes, offset, offset + 1206);
            _decoder.addToCalibrationFrame(data, length);
        }
        bytes = null;
        return _isCalibrated;
    }
    /**
     * Change the number of azimuths required to create a frame. Number needs to be greater than 350.
     * @param num New number of azimuths required. Minimum number is 350.
     */
    public void changeRequiredAzimuths(int num){
        if(num > 350){
            _number_azimuths_in_frame = num;
        }
    }
    /**
     * Scan a full FOV by sampling at least _number_azimuths_in_frame number of times.
     */
    public void scanFullFieldOfView(){
        byte[] data = new byte[1206];
        int[] data_size = {1206};
        System.out.println("VelodyneLidar: Scanning Frame");
        _decoder.ClearFrames();
        _driver.GetPacket(data, data_size);
        _decoder.DecodePacket(data, data_size);
        while((_latestFrame = _decoder.GetLatestFrame(_number_azimuths_in_frame)) == null){
            _driver.GetPacket(data, data_size);
            _decoder.DecodePacket(data, data_size);
        }
        System.out.println("VelodyneLidar: Frame scanned. Number of Azimuths: " + _latestFrame.getNumberOfAzimuthsInFrame());
    }
    /**
     * Scan a full FOV with the specified number of azimuths in it.
     * 
     * @param numberOfAzimuthsInFrame Number of azimuths required to create a full Frame.
     */
    public void updateLatestFrame(int numberOfAzimuthsInFrame){
        numberOfAzimuthsInFrame = (numberOfAzimuthsInFrame < 350)? 350 : numberOfAzimuthsInFrame;
        if((_latestFrame = _decoder.GetLatestFrame(numberOfAzimuthsInFrame)) == null){
            scanFullFieldOfView();
        }
    }
    /**
     * Analyze the most up-to-date frame and look for any obstacles inside of it.
     */
    public void analyzeLatestFrame(){
        
        if(_latestFrame == null){
            scanFullFieldOfView();
        }
        if(_generatePointCloud){
            _obstacleFinder.findObstaclesCartician(_latestFrame);
        }else{
            _obstacleFinder.findObstaclesPolar(_latestFrame);
        }
    }
    /**
     * Clear all buffers within all objects used by lidar.
     */

    public void clearAllDataBuffers(){
        _decoder.UnloadData();
        _latestFrame = null;
        _obstacleFinder.clearObsticlesSeen();
    }
    /**
     * If frame has already been analyzed, then remove and return the closest obsticle to lidar.
     * 
     * @return Obstacle object, NULL if no obstacle found
     */
    public Obstacle getClosestObstacle(){
        if(_obstacleFinder.getNumberOfObticles() == 0){
            analyzeLatestFrame();
        }

        if(_obstacleFinder.getNumberOfObticles() != 0){
            return _obstacleFinder.getLatestObstacleFound();
        }else{
            return null;
        }
    }
    /**
     * Returns true if there are any obtacles in the current frame.
     * 
     * @return True if atleast one obstacle in frame.
     */
    public boolean anyObsticlesInFrame(){
        return (_obstacleFinder.getNumberOfObticles() != 0);
    }
    /**
     * Function meant for debugging. Returns all found obstacles as a string to be displayed
     * during testing.
     * 
     * @return String with information detailing all Obstacles within frame
     */
    public String toString(){
        String s = "";
        Obstacle o;
        while((o = _obstacleFinder.getLatestObstacleFound()) != null){
            s += o.toString() + "\n";
        }
        return s;
    }
    /**
     * Given a hex-file, return its contents as a byte array.
     * 
     * @param file String file directory
     * @return Byte array for contents in file
     */
    private static byte[] readContentIntoByteArray(File file){
        FileInputStream fileInputStream = null;
        byte[] bFile = new byte[(int) file.length()];
        try
        {
           //convert file into array of bytes
           fileInputStream = new FileInputStream(file);
           fileInputStream.read(bFile);
           fileInputStream.close();
        }
        catch (Exception e)
        {
            System.out.println("VelodyneLidar: Warning - Fail to load calibration file");
            e.printStackTrace();
            return null;
        }
        return bFile;
     }

}
