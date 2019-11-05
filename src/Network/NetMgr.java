package Network;

import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.logging.*;

//Socket client class to connect to RPI
public class NetMgr {

    private static final Logger LOGGER = Logger.getLogger(NetMgr.class.getName());

    private String ip;
    private int port;
    public static Socket socket = null;
    private String prevMsg = null;
    private BufferedWriter out;
    private BufferedReader in;
    private int msgCounter = 0;

    private static NetMgr netMgr = null;

    public NetMgr(String ip, int port) {
        this.ip = ip;
        this.port = port;
    }

    public static NetMgr getInstance() {
        if (netMgr == null) {
            netMgr = new NetMgr(NetworkConstants.IP, NetworkConstants.PORT);
        }
        return netMgr;
    }

    //Initiate a connection with RPI if there isn't already one
    public boolean initConn() {
        if (isConnected()) {
            LOGGER.info("Already connected with RPI");
            return true;
        } else {
            try {
                LOGGER.info("Initiating Connection with RPI...");
                socket = new Socket(ip, port);
                out = new BufferedWriter(new OutputStreamWriter(this.socket.getOutputStream()));
                in = new BufferedReader(new InputStreamReader(this.socket.getInputStream()));
                LOGGER.info("Connection with RPI established!");
                return true;
            } catch (UnknownHostException e) {
                LOGGER.warning("Connection Failed: UnknownHostException\n" + e.toString());
                return false;
            } catch (IOException e) {
                LOGGER.warning("Connection Failed: IOException\n" + e.toString());
                return false;
            } catch (Exception e) {
                LOGGER.warning("Connection Failed!\n" + e.toString());
                e.printStackTrace();
                return false;
            }
        }
    }

    //Sending a String type msg through socket
    public boolean send(String msg) {
        try {
            LOGGER.log(Level.FINE, "Sending Message...");
            out.write(msg);
            out.newLine();
            out.flush();
            msgCounter++;
            LOGGER.info(msgCounter + " Message Sent: " + msg);
            prevMsg = msg;
            return true;
        } catch (IOException e) {
            LOGGER.info("Sending Message Failed (IOException)!");
            if (socket.isConnected())
                LOGGER.info("Connection still Established!");
            else {
                while (true) {
                    LOGGER.info("Connection disrupted! Trying to Reconnect!");
                    if (netMgr.initConn()) {
                        break;
                    }
                }
            }
            return netMgr.send(msg);
        } catch (Exception e) {
            LOGGER.info("Sending Message Failed!");
            e.printStackTrace();
            return false;
        }
    }

    public String receive() {
        try {
            LOGGER.log(Level.FINE, "Receving Message...");
            String receivedMsg = in.readLine();
            while (receivedMsg == null || receivedMsg.isEmpty()) {
                receivedMsg = in.readLine();
            }
            LOGGER.info("Received in receive(): " + receivedMsg);
            return receivedMsg;
        } catch (IOException e) {
            LOGGER.info("Receiving Message Failed (IOException)!");
            return receive();
        } catch (Exception e) {
            LOGGER.info("Receiving Message Failed!");
            e.printStackTrace();
        }
        return null;
    }
    public static String rpiImageRec(){
        String msg = "No result";
        try{
            String imgFilePath = "image.png";
            String[] cmdArray = {"python", "mainImageRec.py", "1raw.png"};
            File path = new File("/Volumes/pi/ImageRec");
            Process p = Runtime.getRuntime().exec(cmdArray, null, path);
            BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String result = new String();
            p.waitFor();

            while (true) {
                System.out.println("Trying");
                if((result = in.readLine()) != null){
                    System.out.println(result);
                    break;
                }
            }
            in.close();
            p.destroy();
            System.out.println("hah");
            return result;
        }
        catch(Exception e){
            System.out.println(e);
        }
        return msg;
    }

    //Check if there are existing connection with RPI
    public boolean isConnected() {
        if (socket == null) {
            return false;
        } else {
            return true;
        }
    }

    //Testing connection with RPi
    public static void main(String[] args) throws InterruptedException {
        int i = 3;
        while(i != 0){
            rpiImageRec();
            i--;
        }

    }
    }
//        //Testing MDF String
//        String ip = "192.168.4.1";
//        int port = 5005;
//        Map exploredMap = new Map();
//        MapDescriptor MDF = new MapDescriptor();
//        MDF.loadRealMap(exploredMap, "defaultMap.txt");
//        String data;
//        NetMgr netMgr = new NetMgr(ip, port);
//        netMgr.initConn();


        //test send
//        while (true) {
//            netMgr.send(NetworkConstants.ARDUINO + "w");
//            Thread.sleep(2500);
//            netMgr.send(NetworkConstants.ARDUINO + "a");
//            Thread.sleep(2500);
//            netMgr.send(NetworkConstants.ARDUINO + "d");
//            Thread.sleep(2500);
//        }

//        while (true) {
//            netMgr.send("");
//            do {
//                data = netMgr.receive();
//                System.out.println("\nReceived: " + data);
//            } while (data == null);
//
//
//            data = netMgr.receive();
//            System.out.println("\nReceived: " + data);
//            String msg = "AW3|D|W3|D|W3|D|W3|D|";
//            if (data.equals("checklist")) {
//                netMgr.send(msg);
//            }
//        }
//    }
//}

//            netMgr.closeConn();


//        JSONObject androidJson = new JSONObject();
//
//        // robot
//        JSONArray robotArray = new JSONArray();
//        JSONObject robotJson = new JSONObject()
//                .put("x", 1 + 1)
//                .put("y", 1 + 1)
//                .put("direction", Direction.LEFT.toString().toLowerCase());
//        robotArray.put(robotJson);
//
//        // map
//        String obstacleString = MDF.generateMDFString2(exploredMap);
//        JSONArray mapArray = new JSONArray();
//        JSONObject mapJson = new JSONObject()
//                .put("explored", MDF.generateMDFString1(exploredMap))
//                .put("obstacle", obstacleString)
//                .put("length", obstacleString.length() * 4);
//        mapArray.put(mapJson);
//
//        androidJson.put("map", mapArray).put("robot", robotArray);
//        netMgr.send(androidJson.toString());
//    }
