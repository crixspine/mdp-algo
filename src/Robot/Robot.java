package Robot;
import Map.Map;
import Map.Direction;
import Map.MapDescriptor;
import Map.MapConstants;
import Map.ObsSurface;

import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

import Network.NetMgr;
import Network.NetworkConstants;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

public class Robot {
    /**
     * logger to print log information on robot status, movement, position, etc.
     * Sim to show simulation mode (true) or real run (false)
     * FindingFP to show fastest path (true) or exploration/image recognition (false)
     * ReachGoal to show whether robot has reached Goal (true) or not (false)
     * Pos to reflect position of robot in x,y coordinates
     * Direction to reflect direction that robot is facing
     * Status to show latest successful/unsucessful operation of robot
     * PreMove to show last move of robot
     * SensorMap to show the sensors attached to the robot, identified by unique ID
     * SensorResult to hold the last result obtained from all sensors attached to robot, represented by integer
     * TempStartTime, TempEndTime,TempDiff are variables used to compute delay to simulate steps per second
     * MapDescriptor used to convert maps to appropriate formats
     * ImageCount to keep tack of no of images captured during image recognition
     * ImageHashSet to keep track of images captured with their location and details
     * SurfaceTaken to keep track of all surfaces taken during image recognition
     * AlignCount, TurnAndAlignCount, hasTurnAndAlign to track when to execute calibration
     * DoingImage to show image recognition mode (true) or not (false)
     */
    private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
    private boolean sim;
    private boolean findingFP;
    private boolean reachedGoal;

    private boolean imageRec = false;

    public boolean isDoingImage() {
        return doingImage;
    }

    public void setDoingImage(boolean doingImage) {
        this.doingImage = doingImage;
    }

    private boolean doingImage = false;

    private Point pos;
    private Direction dir;
    private String status;

    private ArrayList<String> sensorList;
    private HashMap<String, Sensor> sensorMap;
    private HashMap<String, Integer> sensorRes;


    public void addCapturedPosition(Point robotPos, Direction robotDir) {
        String dir = robotDir.toString();
        this.capturedPosition.put(robotPos,dir);
    }

    public boolean checkIfCapturedPosition(Point robotPos, Direction robotDir) {
        String dir = robotDir.toString();
        if(capturedPosition.containsKey(robotPos)){
            if(capturedPosition.get(robotPos).equals(dir)){
                return true;
            }
        }
        return false;
    }

    private HashMap<Point, String> capturedPosition;
    private ArrayList<String> capturedImages;
    public void addCapturedImages(String stringId) {
        this.capturedImages.add(stringId);
    }

    public boolean checkIfCapturedImages(String stringId) {
            if(capturedImages.contains(stringId)){
                return true;
            }

        return false;
    }


    private long tempStartTime, tempEndTime, tempDiff;

    private MapDescriptor MDF = new MapDescriptor();

    private int imageCount = 0;
    private HashMap<String, ObsSurface> surfaceTaken = new HashMap<String, ObsSurface>();

    //For image recognition
    //obSurfaces is used to keep track of all obstacle surfaces,
    //so the robot will revisit all obstacle surfaces to take image after exploration.
    //Not using obsSurfaces any more, since we take images during exploration itself
    private ArrayList<ObsSurface> obsSurfaces = new ArrayList<ObsSurface>();

    //To check how many consecutive immediate obstacles R1 and R2 senses
    private int R1count = 0;

    private Point lastR2Pos = null;
    private int alignCount = 0;
    private int turnAndAlignCount = 0;
    private boolean hasTurnAndAlign = false;

    public void setOnObstacle(boolean onObstacle) {
        this.onObstacle = onObstacle;
    }

    private boolean onObstacle = false;
    public int obstacleSide = 0;
    public int obstacleStepsCounter =0;

    public JSONArray getImageResult() {
        return imageResult;
    }

    JSONArray imageResult = new JSONArray();

    public JSONObject getImageJSON(int x, int y, String imageId, Direction dir) {
        JSONObject imageJSON = new JSONObject()
                .put("x", x )
                .put("y", y)
                .put("image ID", imageId)
                .put("direction", dir.toString().toLowerCase());
        return imageJSON;
    }

    /**
     * @param sim Whether robot is in simulation or real run
     * @param findingFP Whether robot is doing fastest path or exploration/image recognition
     * @param row Robot's initial position
     * @param col Robot's initial column
     * @param dir Robot's initial direction
     * Initiate robot
     */
    public Robot(boolean sim, boolean findingFP, int row, int col, Direction dir) {
        this.sim = sim;
        this.findingFP = findingFP;
        this.pos = new Point(col, row);
        this.dir = dir;
        this.reachedGoal = false;  // may need to amend
        this.sensorList = new ArrayList<String>();
        this.sensorMap = new HashMap<String, Sensor>();
        this.sensorRes = new HashMap<String, Integer>();
        this.capturedPosition = new HashMap<Point,String>();
        this.capturedImages = new ArrayList<String>();
        initSensors();
        this.status = "Initialization completed.\n";

    }

    //Convert to string method for robot
    public String toString() {
        return String.format("Robot at %s facing %s\n", pos.toString().substring(14), dir.toString());
    }

    public void setSim(boolean sim) {
        this.sim = sim;
    }

    public void setFindingFP(boolean findingFP) {
        this.findingFP = findingFP;
    }

    public Point getPos() {
        return this.pos;
    }

    public Direction getDir() {
        return this.dir;
    }

    public void setDir(Direction dir) {
        this.dir = dir;
    }

    public String getStatus() {
        return this.status;
    }

    public void setStatus(String status) {
        this.status = status;
    }

    public ArrayList<String> getSensorList() {
        return sensorList;
    }

    public HashMap<String, Sensor> getSensorMap() {
        return sensorMap;
    }

    public Sensor getSensor(String sensorId) {
        return sensorMap.get(sensorId);
    }

    public HashMap<String, Integer> getSensorRes() {
        return sensorRes;
    }

    public HashMap<String, ObsSurface> getSurfaceTaken() {
        return surfaceTaken;
    }

    //Create and set sensors in position and add it to robot's sensorMap
    private void initSensors() {
        int row = this.pos.y;
        int col = this.pos.x;

        //Initialize all 6 sensors (5 short range, 1 long range)
        Sensor[] createdSensors = createSensorsStartPos(row, col);

        addSensorsToRobot();

        //Add all sensors to robot's sensorMap
        createSensorMap(createdSensors);

        //Ensure sensors position is correct w.r.t robot
        if (dir != Direction.UP) {
            rotateSensors(dir);
        }

        this.status = "Sensor initialized\n";
    }

    /**
     * Initialization of the Sensors
     *
     *           ^   ^   ^
     *          SR  SR  SR
     *    < SR [X] [X] [X] SR >
     *    < LR [X] [X] [X]
     *         [X] [X] [X]
     *
     *  LR: Long range sensor
     *  SR: Short range sensor
     *
     *  ID representation:
     *  First letter:   "F" - Forward direction (w.r.t robot)
     *                  "L" - Left direction(w.r.t robot)
     *                  "R" - Right direction (w.r.t robot)
     *
     *  Second letter:  Position of sensor w.r.t other sensors facing that direction
     *                  1 - Leftmost
     *                  2 - Middle
     *                  3 - Rightmost
     *
     *                  (1 will always be closest the forward direction of the robot, followed by 2 and 3)
     *
     * @param row Row coordinates of sensor
     * @param col Col coordinates of sensor
     *
     */

    private Sensor[] createSensorsStartPos(int row, int col){
        //Create 3 x SR front facing sensors
        Sensor[] sensorsCreated = new Sensor[6];
        sensorsCreated[0] = new Sensor(RobotConstants.SENSOR_ID[0], RobotConstants.SHORT_MIN, RobotConstants.SHORT_MAX, row + 1, col - 1,
                Direction.UP);
        sensorsCreated[1] =new Sensor(RobotConstants.SENSOR_ID[1], RobotConstants.SHORT_MIN, RobotConstants.SHORT_MAX, row + 1, col, Direction.UP);
        sensorsCreated[2] =new Sensor(RobotConstants.SENSOR_ID[2], RobotConstants.SHORT_MIN, RobotConstants.SHORT_MAX, row + 1, col + 1,
                Direction.UP);

        //Create 1 x SR right facing sensors
        sensorsCreated[3] =new Sensor(RobotConstants.SENSOR_ID[3], RobotConstants.SHORT_MIN, RobotConstants.SHORT_MAX, row + 1, col + 1,
                Direction.RIGHT);

        //Create 1 x SR left facing sensor
        sensorsCreated[4] =new Sensor(RobotConstants.SENSOR_ID[4], RobotConstants.SHORT_MIN, RobotConstants.SHORT_MAX, row + 1, col - 1,
                Direction.LEFT);
        //Create 1 x LR left facing sensor
        sensorsCreated[5] =new Sensor(RobotConstants.SENSOR_ID[5], RobotConstants.LONG_MIN, RobotConstants.LONG_MAX, row , col - 1,
                Direction.LEFT);

        return sensorsCreated;
    }
    //Add sensors to robot
    private void addSensorsToRobot(){
        this.sensorList.add(RobotConstants.SENSOR_ID[0]);
        this.sensorList.add(RobotConstants.SENSOR_ID[1]);
        this.sensorList.add(RobotConstants.SENSOR_ID[2]);
        this.sensorList.add(RobotConstants.SENSOR_ID[3]);
        this.sensorList.add(RobotConstants.SENSOR_ID[4]);
        this.sensorList.add(RobotConstants.SENSOR_ID[5]);
    }

    //Create a sensor map to keep track of sensor location
    private void createSensorMap(Sensor[] sensors){
        for(Sensor s: sensors){
            sensorMap.put(s.getId(), s);
        }
    }

    //Update sensor's direction when robot moves
    private void setSensorPos(int rowDiff, int colDiff) {
        Sensor s;
        for (int i=0; i< RobotConstants.SENSOR_ID.length; i++) {
            s= sensorMap.get(RobotConstants.SENSOR_ID[i]);
            s.setPos(s.getRow() + rowDiff, s.getCol() + colDiff);
        }
    }

    //Returns true when real run and not simulated
    private boolean isRealRun(){
        return (!this.sim);
    }

    //Update sensor positions on robot when robot turns left or right
    private void updateSensorLocation(Sensor s, Direction turn_dir){
        int newCol, newRow;

        switch (turn_dir){
            case LEFT:
                newCol = pos.x + pos.y - s.getRow();
                newRow = s.getCol() - pos.x + pos.y;
                s.setPos(newRow, newCol);
                break;
            case RIGHT:
                newCol =  s.getRow() - pos.y + pos.x;
                newRow = pos.x +  pos.y - s.getCol();
                s.setPos(newRow, newCol);
                break;
            default:
                LOGGER.warning("No rotation done. Wrong input direction: " + dir);


        }

    }

    //Update sensor positions on sensor map when robot turns left or right
     private void updateSensorMap(Direction turn_dir) {
        switch (turn_dir) {
            case LEFT:
                for (int i = 0; i< RobotConstants.NO_OF_SENSORS; i++) {
                    Sensor s = sensorMap.get(RobotConstants.SENSOR_ID[i]);
                    //Change sensor direction
                    s.setSensorDir(Direction.getAntiClockwise(s.getSensorDir()));
                    //Change sensor position
                    updateSensorLocation(s, turn_dir);
                }
                break;
            case RIGHT:
                for (int i = 0; i< RobotConstants.NO_OF_SENSORS; i++) {
                    //Change sensor direction
                    Sensor s = sensorMap.get(RobotConstants.SENSOR_ID[i]);
                    //Change sensor position
                    s.setSensorDir(Direction.getClockwise(s.getSensorDir()));
                    updateSensorLocation(s, turn_dir);
                }
                break;
            default:
                LOGGER.warning("No rotation done. Wrong input direction: " + turn_dir);
        }
    }

    //Update all sensors' position when robot turns left, right or u-turn (abstraction of updateSensorMap)
    private void rotateSensors(Direction turn_dir) {
        switch (turn_dir) {
            case LEFT:
                updateSensorMap(Direction.LEFT);
                break;
            case RIGHT:
                updateSensorMap(Direction.RIGHT);
                break;
            case DOWN:
                updateSensorMap(Direction.RIGHT);
                updateSensorMap(Direction.RIGHT);
                break;
            default:
                break;
        }
    }
    //Increment alignCount by steps when robot moves
    //To keep track of how many steps taken w/o alignment
    private void incrementAlignCount(int steps){
        alignCount += steps;
    }

    //Converts the movement command (move forward, move back, etc) to string and sends to Arduino for execution
    private void sendCommandToArduino(Command cmd, int steps){
        String cmdStr = getCommand(cmd, steps);
        NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
    }

    //Evaluate multiplier of row increment when robot moves (forward or backward) in the direction it is facing
    private int getRowIncrementForMovement(Command cmd){
        int rowInc = 0;

        switch(this.dir) {
            case UP:
                rowInc = 1;
                break;
            case DOWN:
                rowInc = -1;
                break;
            default:
                break;
        }
        switch(cmd){
            case FORWARD:
                break;
            case BACKWARD:
                rowInc *= -1;
                break;
            default:
                status = String.format("Invalid command: %s! No movement executed.\n", cmd.toString());
//                printer.setText(printer.getText() + status + "\n");
                LOGGER.warning(status);
        }
        return rowInc;

    }

    //Evaluate multiplier of column increment when robot moves (forward or backward) in the direction it is facing
    private int getColIncrementForMovement(Command cmd){
        int colInc = 0;
        switch(this.dir) {
            case LEFT:
                colInc = -1;
                break;
            case RIGHT:
                colInc = 1;
                break;
            default:
                break;
        }
        switch(cmd){
            case FORWARD:
                break;
            case BACKWARD:
                colInc *= -1;
                break;
            default:
                status = String.format("Invalid command: %s! No movement executed.\n", cmd.toString());
                LOGGER.warning(status);
        }
        return colInc;

    }

    //Evaluate multiplier of row increment of sensor when sensing
    private int getRowIncrementForRobotAndSensor(Direction dir){
        int rowInc = 0;
        switch(dir) {
            case UP:
                rowInc = 1;
                break;
            case DOWN:
                rowInc = -1;
                break;
            default:
                break;
        }
        return rowInc;

    }

    //Evaluate multiplier of column increment of sensor when sensing
    private int getColIncrementForRobotAndSensor(Direction dir){
        int colInc = 0;
        switch(dir) {
            case LEFT:
                colInc = -1;
                break;
            case RIGHT:
                colInc = 1;
                break;
            default:
                break;
        }
        return colInc;

    }

    //Simulate delay in simulated exploration to match the steps per second of the robot (preset variable)
    private void simulateDelay(int stepsPerSecond,int steps) throws InterruptedException{
        if (this.sim) {
            this.tempEndTime = System.currentTimeMillis();
            this.tempDiff = RobotConstants.WAIT_TIME / stepsPerSecond * steps - (this.tempEndTime - this.tempStartTime);
            if (this.tempDiff > 0) {
                TimeUnit.MILLISECONDS.sleep(this.tempDiff);
            }
        }
    }

    //Returns true if robot is executing exploration and not fastest path
    private boolean isExploration(){
        return !(this.findingFP);
    }

    //Moves robot in either forward or backward direction with the specified number of steps, and updates exploration map
    public void move(Command cmd, int steps, Map exploredMap, int stepsPerSecond) throws InterruptedException {

        this.tempStartTime = System.currentTimeMillis();

        //Send command to Arduino and update calibration variable if robot is executing real exploration
        if (isRealRun()) {
            sendCommandToArduino(cmd,steps);
            incrementAlignCount(steps);
            lastR2Pos = sensorMap.get("R2").getPos();
        }

        //Determines increment of row and column coordinates with each step depending on direction of robot
        int rowInc = getRowIncrementForMovement(cmd);
        int colInc = getColIncrementForMovement(cmd);

        int newRow = pos.y + rowInc * steps;
        int newCol = pos.x + colInc * steps;

        if(exploredMap.checkValidMove(newRow, newCol)) {
            status = String.format("%s for %d steps\n", cmd.toString(), steps);
            LOGGER.info(status);
            LOGGER.info("row = " + newRow + ", col = " + newCol);

            //Simulate delay to match preset variable of steps per second
            if(sim) {
                simulateDelay(stepsPerSecond, steps);
            }
            //Update robot's position
            this.setPos(newRow, newCol);

            //Update explored map
            if(isExploration()) {
                for (int i = 0; i < steps; i++) {
                    exploredMap.setPassThru(newRow - rowInc * i, newCol - colInc * i);
                }
            }
        }
    }

    //Updates robot direction and all sensors' direction when robot turns
    private void changeRobotAndSensorDirection(Command cmd){
        switch(cmd) {
            case TURN_LEFT:
                this.dir = Direction.getAntiClockwise(dir);
                rotateSensors(Direction.LEFT);
                break;
            case TURN_RIGHT:
                this.dir = Direction.getClockwise(dir);
                rotateSensors(Direction.RIGHT);
                break;
            default:
                this.status = "Invalid command! No movement executed.\n";
                LOGGER.warning(status);
        }
    }

    //Turns robot in either left or right direction, speed adjusted to preset steps per second of robot
    public void turn(Command cmd, int stepsPerSecond) throws InterruptedException {

        tempStartTime = System.currentTimeMillis();
        //Send command to Arduino and update calibration variable if robot is executing real exploration
        if(isRealRun()){
            sendCommandToArduino(cmd,1);
            incrementAlignCount(1);
            lastR2Pos = sensorMap.get("R2").getPos();
        }

        changeRobotAndSensorDirection(cmd);

        status = cmd.toString() + "\n";
        LOGGER.info(status);
        LOGGER.info(pos.toString());
    }

    //Updates robot's position and cells of starting position on explored map
    public void setStartPos(int row, int col, Map exploredMap) {
        setPos(row, col);
        exploredMap.setAllExplored(false);
        exploredMap.setAllMoveThru(false);
        for (int r = row - 1; r <= row + 1; r++) {
            for (int c = col - 1; c <= col + 1; c++) {
                exploredMap.getCell(r, c).setExplored(true);
                exploredMap.getCell(r, c).setMoveThru(true);
            }
        }
    }

    //Updates robot position and sensor's position when robot changes position (due to movement, etc)

    private void setPos(int row, int col) {
        int colDiff = col - this.pos.x;
        int rowDiff = row - this.pos.y;
        this.pos.setLocation(col, row);
        setSensorPos(rowDiff, colDiff);
    }

    //Parse start point sent from Android
     public Point parseStartPointJson(String jsonMsg) {
        if (jsonMsg.contains(NetworkConstants.START_POINT_KEY)) {
            JSONObject startPointJson = new JSONObject(new JSONTokener(jsonMsg));
            return (new Point((int) startPointJson.get("x") , (int) startPointJson.get("y")));
        }
        else {
            LOGGER.warning("Not a start point msg. Return null.");
            return null;
        }
    }

    //Parse way point sent from Android
    public Point parseWayPointJson(String jsonMsg) {

        //Confirm message is specified as way point message
        //{"x":1,"y":1,"waypoint":"true"}
        if (jsonMsg.contains(NetworkConstants.WAY_POINT_KEY)) {
            JSONObject wayPointJson = new JSONObject(new JSONTokener(jsonMsg));
            return (new Point((int) wayPointJson.get("x"), (int) wayPointJson.get("y")));
        }
        else {
            LOGGER.warning("Not a start point msg. Return null.");
            return null;
        }
    }

    //Obtain sensor result from Arduino (through Rasberry Pi) and updates sensor result in robot
    private HashMap<String, Integer> updateSensorRes(String msg) {
        //First sensor should be "F1"
        if (msg.charAt(0) != 'F') {
            return null;
        }

        else {
            String[] sensorStrings = msg.split("\\|");
            for (String sensorStr: sensorStrings) {
                String[] sensorInfo = sensorStr.split(",");
                System.out.println(sensorStr);
                String sensorID = sensorInfo[0];
                int result = Integer.parseInt(sensorInfo[1]);
                //Validate that result obtain is within sensor range, otherwise set as -1
                if (result >= sensorMap.get(sensorID).getMinRange() && result <= sensorMap.get(sensorID).getMaxRange()) {
                    sensorRes.put(sensorID, result);
                }
                else {
                    sensorRes.put(sensorID, -1);
                }
            }
            return sensorRes;
        }
    }

    //Obtain sensor result from realMap in simulation mode (no real sensor needed)
    public HashMap<String, Integer> updateSensorRes(Map realMap) {
        int obsBlock;
        for(String sname: RobotConstants.SENSOR_ID) {

            obsBlock = sensorMap.get(sname).detect(realMap);
            sensorRes.put(sname, obsBlock);
        }
        return sensorRes;
    }

    //Set counter for temporal right alignment
    public void setR1count(int R1count){
        this.R1count = R1count;
    }

    //Check if point 1 is same as point 2 lol
    public boolean isSamePoint(Point p1, Point p2){
        if((p1.x == p2.x) && (p1.y == p2.y)){
            return true;
        }
        return false;
    }

    //Sensing surrounding cells for obstacles in simulation mode
    public ArrayList<ObsSurface> sense(Map exploredMap, Map realMap) {
        HashMap<String, Integer> sensorResult = completeUpdateSensorResult(realMap);
        updateMap(exploredMap, sensorResult);

        if (isRealRun()) {
            //Send updated map to Android
            sendAndroid(exploredMap);

            //If robot is adjacent to right wall, will use normal calibrate after steps
            if(isRightHuggingWall() && alignCount> RobotConstants.CALIBRATE_AFTER){
                align_right(exploredMap, realMap);
                R1count = 0;
                alignCount = 0;
            }
            else if(R1count == 3){
                align_right1(exploredMap, realMap);
            }

        }

        return null;
    }

    //Update sensorRes but not the map. No alignment as well. Send image as well.
    public void senseWithoutMapUpdateAndAlignment(Map exploredMap, Map realMap) {
        completeUpdateSensorResult(realMap);
        if (isRealRun()) {
            sendAndroid(exploredMap);
        }
    }

    //Sense map without alignment
    private void senseWithoutAlign(Map exploredMap, Map realMap) {
        HashMap<String, Integer> sensorResult = completeUpdateSensorResult(realMap);
        updateMap(exploredMap, sensorResult);

        // Send updated map to android
        if (isRealRun()) {
            sendAndroid(exploredMap);
        }
    }

    //Updates results from sensor in both simulated and real exploration
    private HashMap<String, Integer> completeUpdateSensorResult(Map realMap) {
        HashMap<String, Integer> sensorResult;

        if(sim) {
            sensorResult = updateSensorRes(realMap);
        }
        else {
            //Receive sensor result from Arduino
            String msg = NetMgr.getInstance().receive();
            sensorResult = updateSensorRes(msg);
        }
        return sensorResult;
    }

    //Obstaclesurfaces here
    public void removeObsSurfaces(Point point){
        if(!imageRec) {
            for (int i = 0; i < obsSurfaces.size(); i++) {
                if (isSamePoint(obsSurfaces.get(i).getPos(), point)) {
                    obsSurfaces.remove(obsSurfaces.get(i));
                }
            }
        }
    }

    //Update the map with result obtained from sensor
    public void updateMap(Map exploredMap, HashMap<String, Integer> sensorResult) {
        int obsBlock;
        int rowInc, colInc, tempRow, tempCol;

        if(sensorResult == null) {
            LOGGER.warning("Invalid msg. Map not updated");
            return;
        }

        for(String sname: RobotConstants.SENSOR_ID) {
            Sensor s = sensorMap.get(sname);
            obsBlock = sensorResult.get(sname);

            // Assign the rowInc and colInc based on sensor direction
            rowInc = getRowIncrementForRobotAndSensor(s.getSensorDir());
            colInc = getColIncrementForRobotAndSensor(s.getSensorDir());

            if(s.getId() == "L1") {
                LOGGER.info(s.getId());
//                //Update cells as clear if first two blocks are already cleared
                if (obsBlock < s.getMinRange() || obsBlock > s.getMaxRange() && lastR2Pos != null && isSamePoint(lastR2Pos, s.getPos())) {
                    int cell1Row, cell2Row, cell1Col, cell2Col;
                    tempRow = s.getRow() + rowInc * s.getMinRange();
                    tempCol = s.getCol() + colInc * s.getMinRange();
                    if (exploredMap.checkValidCell(tempRow, tempCol)) {
                        cell1Row = s.getRow() + rowInc;
                        cell1Col = s.getCol() + colInc;
                        cell2Row = s.getRow() + rowInc * 2;
                        cell2Col = s.getCol() + colInc * 2;
                        //If first two cells are explored and not obstacle
                        if (exploredMap.getCell(cell1Row, cell1Col).isExplored() && !exploredMap.getCell(cell1Row, cell1Col).isObstacle()) {
                            if (exploredMap.getCell(cell2Row, cell2Col).isExplored() && !exploredMap.getCell(cell2Row, cell2Col).isObstacle()) {
                                //Set remaining cells in line of sight as non obstacles
                                for (int i = s.getMinRange(); i <= s.getMaxRange(); i++) {
                                    tempRow = s.getRow() + rowInc * i;
                                    tempCol = s.getCol() + colInc * i;
                                    if (exploredMap.checkValidCell(tempRow, tempCol)) {
                                        exploredMap.getCell(tempRow, tempCol).setExplored(true);
                                        exploredMap.getCell(tempRow, tempCol).setObstacle(false);
                                        if(!imageRec) {
                                            removeObsSurfaces(new Point(tempRow, tempCol));
                                        }
                                        exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), false);
                                    } else {
                                        break;
                                    }

                                }
                                exploredMap.reinitializeVirtualWall();
                            }
                        }
                    }


                } else {
                    for (int j = s.getMinRange(); j <= s.getMaxRange(); j++) {
                        tempRow = s.getRow() + rowInc * j;
                        tempCol = s.getCol() + colInc * j;
                        if (exploredMap.checkValidCell(tempRow, tempCol)) {

                            //Update specified cell when identified as obstacle
                            //Will not update as obstacle if area has been moved through
                            if (j == obsBlock && !exploredMap.getCell(tempRow, tempCol).isMoveThru()) {
                                if (!sim) {
                                    int tempRow1, tempCol1;
                                    exploredMap.getCell(tempRow,tempCol).setExplored(true);
                                    exploredMap.getCell(tempRow, tempCol).setObstacle(true);
                                    if(!imageRec) {
//                                        addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                                    }
                                    for (int i = 1; i < obsBlock; i++) {
                                        tempRow1 = s.getRow() + rowInc * i;
                                        tempCol1 = s.getCol() + colInc * i;
                                        exploredMap.getCell(tempRow1, tempCol1).setExplored(true);
                                        exploredMap.getCell(tempRow1, tempCol1).setObstacle(false);
                                        if(!imageRec) {
                                            removeObsSurfaces(new Point(tempRow, tempCol));
                                        }
                                        exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), false);
                                    }
                                } else {
                                    int tempRow2, tempCol2;
                                    boolean obstacleInLine = false;
                                    for (int i = 1; i < obsBlock; i++) {

                                        tempRow2 = s.getRow() + rowInc * i;
                                        tempCol2 = s.getCol() + colInc * i;
                                        if (exploredMap.getCell(tempRow2, tempCol2).isObstacle()) {
                                            obstacleInLine = true;
                                            break;
                                        }
                                    }
                                    if (!obstacleInLine) {
                                        exploredMap.getCell(tempRow, tempCol).setObstacle(true);
                                        exploredMap.getCell(tempRow, tempCol).setExplored(true);
//                                        System.out.println("temp row: " + tempRow);
//                                        System.out.println("temp col: " + tempCol);
                                        if(!imageRec) {
//                                            addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                                        }
                                        for (int i = 1; i < obsBlock; i++) {
                                            tempRow2 = s.getRow() + rowInc * i;
                                            tempCol2 = s.getCol() + colInc * i;
                                            exploredMap.getCell(tempRow2, tempCol2).setExplored(true);
                                            exploredMap.getCell(tempRow2, tempCol2).setObstacle(false);
                                            if(!imageRec) {
                                                removeObsSurfaces(new Point(tempRow, tempCol));
                                            }
                                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow2, tempCol2), false);
                                        }
                                    }
                                }
                                exploredMap.reinitializeVirtualWall();
                            }
                        }
                    }
                }
            }
            else {
                LOGGER.info(s.getId());
                //Check for every block within sensor's valid range
                for (int j = s.getMinRange(); j <= s.getMaxRange(); j++) {

                    tempRow = s.getRow() + rowInc * j;
                    tempCol = s.getCol() + colInc * j;

                    // Check whether the block is a valid block
                    if (exploredMap.checkValidCell(tempRow, tempCol)) {
                        exploredMap.getCell(tempRow, tempCol).setExplored(true);

                        //Update specified cell when identified as obstacle
                        //Will not update as obstacle if area has been moved through
                        if (j == obsBlock && !exploredMap.getCell(tempRow, tempCol).isMoveThru()) {
                            exploredMap.getCell(tempRow, tempCol).setObstacle(true);
                            if(!imageRec) {
//                                addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                            }
                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), true);
                            exploredMap.reinitializeVirtualWall();
                            if(s.getId() == "R1" && obsBlock == 1){
                                System.out.println("Robot Position: " + this.getPos().toString());
                                System.out.println("R1 Counters: " + R1count);
                                R1count++;
                            }
                            break;
                        }

                        //Previous detected obstacle is wrongly detected; reset the cell and virtual walls
                        else if (j != obsBlock && exploredMap.getCell(tempRow, tempCol).isObstacle()) {      // (3)
                            exploredMap.getCell(tempRow, tempCol).setObstacle(false);
                            if(!imageRec) {
                                removeObsSurfaces(new Point(tempRow, tempCol));
                            }
                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), false);
                            exploredMap.reinitializeVirtualWall();
                            if(s.getId() == "R1"){
                                R1count=0;
                            }
                        }
                    } else {
                        if(s.getId() == "R1"){
                            R1count=0;
                        }
                        break;
                    }

                }
            }
        }
    }

    //Translate direction and position of robot into JSON for transmission
    private JSONArray getRobotArray() {

        JSONArray robotArray = new JSONArray();
        JSONObject robotJson = new JSONObject()
                .put("x", pos.x )
                .put("y", pos.y)
                .put("direction", dir.toString().toLowerCase());
        robotArray.put(robotJson);
        return robotArray;
    }

    //Translate map into JSON array for transmission
    private JSONArray getMapArray(Map exploredMap) {
        String obstacleString = MDF.generateMDFString2(exploredMap);
        JSONArray mapArray = new JSONArray();
        JSONObject mapJson = new JSONObject();
        mapJson.put("explored", MDF.generateMDFString1(exploredMap));
        mapJson.put("obstacle", obstacleString);
        mapJson.put("length", obstacleString.length() * 4);
        mapArray.put(mapJson);
        return mapArray;
    }

    //Translate status of robot into JSONArray
    private JSONArray getStatusArray() {
        JSONArray statusArray = new JSONArray();
        JSONObject statusJson = new JSONObject()
                .put("status", status.replaceAll("\\n",""));
        statusArray.put(statusJson);
        return statusArray;
    }

    //Send robot's direction and position, current explored environment of arena to Android
    public void sendAndroid(Map exploredMap) {
        JSONObject androidJson = new JSONObject();

        androidJson.put("robot", getRobotArray());
        androidJson.put("map", getMapArray(exploredMap));
        androidJson.put("status", getStatusArray());
        androidJson.put("image", getImageResult());
        //Might be very long
        NetMgr.getInstance().send(NetworkConstants.ANDROID + androidJson.toString() + "\n");

    }

    public String takeImage() {
        System.out.println("Send RPI taking image");

        NetMgr.getInstance().send(NetworkConstants.RPI_TAKEIMG);
        String msg = NetMgr.getInstance().receive();
        while(msg == null){
            msg = NetMgr.getInstance().receive();
        }
        return msg;

    }

    //For calibration using 1 sensor
    public boolean checkFrontForSingleObstacle(Map exploredMap, Direction dir){
        Point F1= new Point();
        Point F2= new Point();
        Point F3= new Point();
        switch(dir){
            case UP: {
                F1.x = this.getPos().x -1;
                F1.y = this.getPos().y +2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y+2;
                F3.x = this.getPos().x +1;
                F3.y = this.getPos().y+2;
                break;
            }
            case RIGHT:{
                F1.x = this.getPos().x +2;
                F1.y = this.getPos().y +1;
                F2.x = this.getPos().x +2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x +2;
                F3.y = this.getPos().y-1;
                break;
            }
            case DOWN:{
                F1.x = this.getPos().x +1;
                F1.y = this.getPos().y -2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y -2;
                F3.x = this.getPos().x -1;
                F3.y = this.getPos().y -2;
                break;
            }
            case LEFT:{
                F1.x = this.getPos().x -2;
                F1.y = this.getPos().y -1;
                F2.x = this.getPos().x -2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x -2;
                F3.y = this.getPos().y+1;
                break;
            }
        }

        if(exploredMap.checkValidCell(F1.y, F1.x) && exploredMap.checkValidCell(F3.y,F3.x) && exploredMap.checkValidCell(F2.y,F2.x)){
            return (exploredMap.getCell(F1.y, F1.x).isObstacle() || exploredMap.getCell(F3.y, F3.x).isObstacle() ||exploredMap.getCell(F2.y, F2.x).isObstacle());
        }
        return false;
    }

    private boolean checkFrontForObstacleOrRightWall(Map exploredMap){
        Point F1= new Point();
        Point F2= new Point();
        Point F3= new Point();
        switch(this.getDir()){
            case UP: {
                F1.x = this.getPos().x -1;
                F1.y = this.getPos().y +2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y+2;
                F3.x = this.getPos().x +1;
                F3.y = this.getPos().y+2;
                break;
            }
            case RIGHT:{
                F1.x = this.getPos().x +2;
                F1.y = this.getPos().y +1;
                F2.x = this.getPos().x +2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x +2;
                F3.y = this.getPos().y-1;
                break;
            }
            case DOWN:{
                F1.x = this.getPos().x -1;
                F1.y = this.getPos().y -2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y -2;
                F3.x = this.getPos().x +1;
                F3.y = this.getPos().y -2;
                break;
            }
            case LEFT:{
                F1.x = this.getPos().x -2;
                F1.y = this.getPos().y +1;
                F2.x = this.getPos().x -2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x -2;
                F3.y = this.getPos().y+1;
                break;
            }
        }

        int F2row = this.getSensor("F2").getRow();
        int F2col = this.getSensor("F2").getCol();
        if(F2row == 0 || F2row == MapConstants.MAP_HEIGHT-1 || F2col == 0 || F2col == MapConstants.MAP_WIDTH-1) {
            return true;
        }
        else if(exploredMap.checkValidCell(F1.y, F1.x) && exploredMap.checkValidCell(F3.y,F3.x)){
            return exploredMap.getCell(F1.y, F1.x).isObstacle() && exploredMap.getCell(F3.y, F3.x).isObstacle();
        }
        return false;
    }

    private boolean checkRightisObstacleOrWall(Map exploredMap){
        Point F1= new Point();
        Point F2= new Point();
        Point F3= new Point();
        switch(this.getDir()){
            case RIGHT: {
                F1.x = this.getPos().x -1;
                F1.y = this.getPos().y +2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y+2;
                F3.x = this.getPos().x +1;
                F3.y = this.getPos().y+2;
                break;
            }
            case DOWN:{
                F1.x = this.getPos().x +2;
                F1.y = this.getPos().y +1;
                F2.x = this.getPos().x +2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x +2;
                F3.y = this.getPos().y-1;
                break;
            }
            case LEFT:{
                F1.x = this.getPos().x -1;
                F1.y = this.getPos().y -2;
                F2.x = this.getPos().x;
                F2.y = this.getPos().y -2;
                F3.x = this.getPos().x +1;
                F3.y = this.getPos().y -2;
                break;
            }
            case UP:{
                F1.x = this.getPos().x -2;
                F1.y = this.getPos().y +1;
                F2.x = this.getPos().x -2;
                F2.y = this.getPos().y;
                F3.x = this.getPos().x -2;
                F3.y = this.getPos().y+1;
                break;
            }
        }
        if(isRightHuggingWall()){
            return true;
        }
        else if(exploredMap.checkValidCell(F1.y, F1.x) && exploredMap.checkValidCell(F3.y,F3.x)){
            return exploredMap.getCell(F1.y, F1.x).isObstacle() && exploredMap.getCell(F3.y, F3.x).isObstacle();
        }
        return false;
    }

    //Calibrate robot's direction using front sensors, returns true if obstacles are detected
    public boolean align_front(Map exploredMap, Map realMap) {
        //Robot directly in front of obstacle/wall

        if(checkFrontForObstacleOrRightWall(exploredMap)){
            if(!sim){
                String cmdStr = getCommand(Command.ALIGN_FRONT, 1);

                NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
                senseWithoutAlign(exploredMap, realMap);
            }
            status = "Aligning Front\n";
            LOGGER.info(status);
            turnAndAlignCount = 0;
            return true;
        }
        else if(checkFrontForSingleObstacle(exploredMap,this.getDir())){
            if(!sim) {
                //alignment using 1 sensor
                align_front1(exploredMap, realMap);
            }

            status = "Aligning Front 1\n";
            LOGGER.info(status);
            return true;
        }
        return false;
    }
    //Calibrate robot's direction using one of front sensors
    public void align_front1(Map exploredMap, Map realMap) {
        //Robot directly in front of obstacle/wall
            // Send align front command to Arduino
            String cmdStr = getCommand(Command.ALIGN_FRONT1, 1);  // steps set to 0 to avoid appending to cmd
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            senseWithoutAlign(exploredMap, realMap);
            turnAndAlignCount = 0;
        }

    //Calibrate robot's direction using front sensors
    public void align_front_no_update() {
        //Robot directly in front of obstacle/wall
        if (sensorRes.get("F1") == 1 && sensorRes.get("F3") == 1) {
            // Send align front command to Arduino
            String cmdStr = getCommand(Command.ALIGN_FRONT, 1);  // steps set to 0 to avoid appending to cmd

            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            status = "Aligning Front\n";
            LOGGER.info(status);

        }
        else{
            align_front1_no_update();
        }

    }
    public void align_front1_no_update() {
        //Robot directly in front of obstacle/wall
        // Send align front command to Arduino
        String cmdStr = getCommand(Command.ALIGN_FRONT1, 1);  // steps set to 0 to avoid appending to cmd

        NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
        status = "Aligning Front 1\n";
        LOGGER.info(status);
    }
    //Calibrate robot's direction using right sensors
    public void align_right(Map exploredMap, Map realMap) {
        if(checkRightisObstacleOrWall(exploredMap)) {
            String cmdStr = getCommand(Command.ALIGN_RIGHT, 1);
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            alignCount = 0;
            status = String.format("Aligning Right: %d\n", 1);
            LOGGER.info(status);

            senseWithoutAlign(exploredMap, realMap);
        }
    }
    public void align_right1(Map exploredMap, Map realMap) {

            String cmdStr = getCommand(Command.ALIGN_RIGHT, 1);
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            alignCount = 0;
            status = String.format("Aligning Right: %d\n", 1);
            LOGGER.info(status);

            senseWithoutAlign(exploredMap, realMap);
    }

    //Robot is right hugging the wall if the right sensor position is equal to
    public boolean isRightHuggingWall() {

        Point R1_pos = sensorMap.get("R1").getPos();
        Point R2_pos = sensorMap.get("R2").getPos();
        if (R1_pos.x == 0 && R2_pos.x ==2 ){
            return true;
        }
        if(R1_pos.x == MapConstants.MAP_WIDTH - 1 && R2_pos.x == MapConstants.MAP_WIDTH - 3){
            return true;
        }
        if(R1_pos.y == 0 && R2_pos.y == 2){
            return true;
        }
        if(R1_pos.y == MapConstants.MAP_HEIGHT - 1 && R2_pos.y == MapConstants.MAP_HEIGHT- 3)
        {
            return true;
        }
        return false;
    }

    //Robot is right hugging the wall if the right sensor position is equal to
    public boolean isFacingWall() {
        Point F1_pos = sensorMap.get("F1").getPos();
        Point F2_pos = sensorMap.get("F2").getPos();
        if (F1_pos.y == 0 && F2_pos.y ==0 ){
            return true;
        }
        if(F1_pos.x == MapConstants.MAP_WIDTH - 1 && F2_pos.x == MapConstants.MAP_WIDTH - 1){
            return true;
        }
        if(F1_pos.x == 0 && F2_pos.x == 0){
            return true;
        }
        return (F1_pos.y == MapConstants.MAP_HEIGHT-1 && F2_pos.y == MapConstants.MAP_HEIGHT-1);
    }

    public int getAlignCount() {
        return alignCount;
    }

    public void setAlignCount(int alignCount) {
        this.alignCount = alignCount;
    }

    public void setHasTurnAndAlign(boolean canTurn) {
        this.hasTurnAndAlign = canTurn;
    }

    public int getImageCount() {
        return imageCount;
    }

    public void setImageCount(int count) {
        this.imageCount = count;
    }

    //Convert the command to a string format for sending to RPi/Arduino
    public String getCommand(Command cmd, int steps) {
        StringBuilder cmdStr = new StringBuilder();

        cmdStr.append(Command.ArduinoMove.values()[cmd.ordinal()]);

        cmdStr.append(steps);
        cmdStr.append('|');

        return cmdStr.toString();
    }

    public String rpiImageRec(String imgFilePath){
        System.out.println("Calling RPI to image rec after taking image");
        String msg = "no result";
        try{

            String[] cmdArray = {"python", "mainImageRec.py", imgFilePath};
            File path = new File("Y:/ImageRec/");
            Process p = Runtime.getRuntime().exec(cmdArray, null, path);

            BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String result;
            p.waitFor();

            //while (true) {
                System.out.println("Attempting to get image result from Python Script");
                if((result = in.readLine()) != null){
                    System.out.println(result);
                    in.close();
                    p.destroy();
                    return result;
                }
                return "None, None";
        }
        catch(Exception e){
            System.out.println(e);
        }
        return msg;
    }
}


