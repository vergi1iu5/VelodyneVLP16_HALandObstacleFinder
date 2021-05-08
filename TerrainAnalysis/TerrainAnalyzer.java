package com.AURMC.lib.TerrainAnalysis;
import com.AURMC.lib.Hardware.VelodyneLidarHDL.PacketDecoder.HDLFrame;

public interface TerrainAnalyzer {

    //private ArrayList<Obstacle> foundObstacles = new ArrayList<Obstacle>();
    /*
    I Think that the simples way to do this is by keeping a single obsticle-buffer. 
    Everytime we do a call to the find obstacle function, we provide it a HDLFrame from 
    which we find all obsticles and add them to the ArrayList mentioned on top. Calling
    the function again will clear the array and once again find all obsticles in the HDLFrame.
    Every frame comes with a timestamp which we can use to see if frame has already been analysized.
    */
    public void findObstaclesCartician(HDLFrame frame);
    public void findObstaclesPolar(HDLFrame frame);
    /*
    Easy way to determine if any obsticles have been found
    */
    public int getNumberOfObticles();
    //Return the obsticle closer to the robot.
    public Obstacle getLatestObstacleFound();

    public void clearObsticlesSeen();

    public void addObstacle(Obstacle o);

    //
}
