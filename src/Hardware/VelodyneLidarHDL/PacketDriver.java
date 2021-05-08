// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package Hardware.VelodyneLidarHDL;

import java.net.DatagramSocket;
import java.io.IOException;
import java.net.DatagramPacket;

/**
 * PacketDriver implements the dataGather interface for retriving packets from an RS32 connection
 *
 * <p>The PacketDriver is intended to be a subclass for a user implementing the VelodyneLidar Class
 *
 * <p>getPacket() function gets called by VelodyneLidar class to retreive the latest data packet sent by lidar.
 */
public class PacketDriver{

    private int _port;
    private byte[] _rx_buffer = new byte[1206];
    private DatagramSocket _socket;
    private DatagramPacket _packet;

    public PacketDriver(){

    }
    /**
     * Constructor for PacketDriver.
     *
     * @param[in] port The port number to connect to.
     */
    public PacketDriver(int port){
        //Attempt to bind to provided socket port number
        try{
            this._socket = new DatagramSocket(port);
            this._packet = new DatagramPacket(_rx_buffer, _rx_buffer.length);
        }catch(IOException e1){ //Second attempt
            System.out.println("PacketDriver: Error binding to socket - " + e1.getMessage() + ". Trying once more...");
            try{
                this._socket = new DatagramSocket(port);
                this._packet = new DatagramPacket(_rx_buffer, _rx_buffer.length);
            }catch(IOException e2){ //Failed to bind to socket
                System.out.println("PacketDriver: Error binding to socket - " + e2.getMessage() + ". Failed!");
                return;
            }
        }

        System.out.println("PacketDriver: Success binding to Velodyne Socket!");
        return;
    }
    /**
     * Overwritten finalizer to ganrantee socket gets closed
     */
    @Override
    public void finalize(){
        _socket.close();
        System.out.println("Closed Velodyne Socket!");
    }
    /**
     * Initialize PacketDriver if class constructed using default constructor
     *
     * @param[in] port The port number to connect to.
     */
    public void InitPacketDriver(int port){
        this._port = port;
        //Attempt to bind to provided socket number
        try{
            this._socket = new DatagramSocket(port);
            this._packet = new DatagramPacket(_rx_buffer, _rx_buffer.length);
        }catch(IOException e1){ //Second attempt
            System.out.println("PacketDriver: Error binding to socket - " + e1.getMessage() + ". Trying once more...");
            try{
                this._socket = new DatagramSocket(port);
                this._packet = new DatagramPacket(_rx_buffer, _rx_buffer.length);
            }catch(IOException e2){//Failed connection
                System.out.println("PacketDriver: Error binding to socket - " + e2.getMessage() + ". Failed!");
                return;
            }
        }

        System.out.println("PacketDriver: Success binding to Velodyne Socket!");
        return;
    }
    /**
     * Funtion for getting a single packet through binded socket
     *
     * @param[out] data Byte buffer to receive packet in
     * @param[in] data_length size for provided data buffer
     */
    public boolean GetPacket(byte[] data, int[] data_length){
        try{ //Attempt to receive packet from socket connection
            _socket.receive(_packet);
            _rx_buffer = _packet.getData();
            System.arraycopy(_rx_buffer, 0, data, 0, data_length[0]);
            return (true); //Successfully retreived data packet
        }catch(IOException e){
            //Failed to receive packet
            System.out.println("PacketDriver: Error receiving packet - " + e.getMessage() + ".");
            return (false);
        }
    }

}