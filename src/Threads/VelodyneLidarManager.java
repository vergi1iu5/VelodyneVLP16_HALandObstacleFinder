// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package Threads;

import Hardware.VelodyneLidarHDL.VelodyneLidar;
import TerrainAnalysis.Obstacle;

import java.util.concurrent.BlockingQueue;
import java.util.ArrayList;
/**
 * VelodyneLidarManger class to be operated in a multi-thread enviroment. Once the thread is started,
 * it waits for a message in the input blocking queue. This message contains the number of azimuths to be 
 * used in analyzing a frame. Once done, it places an ArrayList message on the output blocking queue.
 * 
 * <p>Intantiate class with a BlockingQueue for request and another for responses.
 * <p>Ex:
 *  BlockingQueue<Integer> requests = new LinkedBlockingQueue<>();
 *  BlockingQueue<SquareResult> replies = new LinkedBlockingQueue<>();
 *
 *  VelodyneLidarManager lidarManager = new VelodyneLidarManager(requests, replies);
 *  lidarManager.start();
 *
 *  try {
 *      // make a request with 5000 azimuths in the analyzed frame
 *      requests.put(5000);
 *      // ... maybe do something concurrently ...
 *      // read the reply
 *       ArrayList<Obstacle> obtacles = replies.take();
 *      //Do something with all obtacles in the array.
 *  } catch (InterruptedException ie) {
 *      ie.printStackTrace();
 *  }
 */
public class VelodyneLidarManager {
    private final BlockingQueue<Integer> _in;
    private final BlockingQueue<ArrayList<Obstacle>> _out;
    private final VelodyneLidar _lidar;
    /**
     * Class constructor. Initializes lidar to the parameters that better meet our team
     * needs. Change the paramenetrs in the VelodyneLidar constructor.
     * 
     * @param requests Will contain the number of azimuths the client wants in each scan
     * @param replies Will contain an array of all th obstacles found inside of the frame analyzed.
     */
    public VelodyneLidarManager(BlockingQueue<Integer> requests,
        BlockingQueue<ArrayList<Obstacle>> replies){
        this._in = requests;
        this._out = replies;
        this._lidar = new VelodyneLidar(0.07, 0.00, 100, 5000, false);
        _lidar.calibrateLidar();
    }
    /**
     * Start thread and wait for inputs in the _in stream.
     */
    public void start(){
        new Thread(new Runnable(){
            public void run(){
                while (true){
                    try{
                        //Block until a request arrives
                        int num_azimuths_Request = _in.take();
                        if(num_azimuths_Request < 0){break;} //Request to end thread
                    
                        _lidar.updateLatestFrame(num_azimuths_Request);
                        _lidar.analyzeLatestFrame();
                        ArrayList<Obstacle> ret = new ArrayList<Obstacle>();
                        while(_lidar.anyObsticlesInFrame()){
                            ret.add(_lidar.getClosestObstacle());
                        }
                        ret.add(null);
                        _out.put(ret);
                    }catch(InterruptedException ie){
                        ie.printStackTrace();
                    }
                }
            }
        }).start();
    }

}

/*
Example of how it would be used. Somewhere where you want to call for obtacles:

    BlockingQueue<Integer> requests = new LinkedBlockingQueue<>();
    BlockingQueue<SquareResult> replies = new LinkedBlockingQueue<>();

    VelodyneLidarManager lidarManager = new VelodyneLidarManager(requests, replies);
    lidarManager.start();

    try {
        // make a request with 5000 azimuths in the analyzed frame
        requests.put(5000);
        // ... maybe do something concurrently ...
        // read the reply
        ArrayList<Obstacle> obtacles = replies.take();
        //Do something with all obtacles in the array.
    } catch (InterruptedException ie) {
        ie.printStackTrace();
    }

*/
