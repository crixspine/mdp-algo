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
import java.util.HashSet;
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

    public boolean isImageRec() {
        return imageRec;
    }

    public void setImageRec(boolean imageRec) {
        this.imageRec = imageRec;
    }

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

    public Command getPreMove() {
        return preMove;
    }

    public void setPreMove(Command preMove) {
        this.preMove = preMove;
    }

    private Command preMove = Command.INITIAL_CALIBRATE;

    private ArrayList<String> sensorList;
    private HashMap<String, Sensor> sensorMap;
    private HashMap<String, Integer> sensorRes;

    private long tempStartTime, tempEndTime, tempDiff;

    private MapDescriptor MDF = new MapDescriptor();

    private int imageCount = 0;
    private HashSet<String> imageHashSet = new HashSet<String>();
    private HashMap<String, ObsSurface> surfaceTaken = new HashMap<String, ObsSurface>();

    public ArrayList<ObsSurface> getObsSurfaces() {
        return obsSurfaces;
    }

    public void setObsSurfaces(ArrayList<ObsSurface> obsSurfaces) {
        this.obsSurfaces = obsSurfaces;
    }

    //For image recognition
    private ArrayList<ObsSurface> obsSurfaces = new ArrayList<ObsSurface>();

    //To check how many consecutive immediate obstacles R1 and R2 senses
    private int R1count = 0;

    private Point lastR2Pos = null;
    private int alignCount = 0;
    private int turnAndAlignCount = 0;
    private boolean hasTurnAndAlign = false;

    public boolean isOnObstacle() {
        return onObstacle;
    }

    public void setOnObstacle(boolean onObstacle) {
        this.onObstacle = onObstacle;
    }

    private boolean onObstacle = false;
    public int obstacleSide = 0;
    public int obstacleStepsCounter =0;

    public JSONArray getImageResult() {
        return imageResult;
    }

    public void setImageResult(JSONArray imageResult) {
        this.imageResult = imageResult;
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



    //Removed doingImage

    /**
     *
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
        initSensors();
        this.status = "Initialization completed.\n";

    }

    /**
     * Convert to string method for robot
     */
    @Override
    public String toString() {
        return String.format("Robot at %s facing %s\n", pos.toString().substring(14), dir.toString());
    }

    /**
     * Getters and setters for all attributes
     */

    // Getters and setters
    public boolean getSim() {
        return this.sim;
    }

    public void setSim(boolean sim) {
        this.sim = sim;
    }

    public boolean isFindingFP() {
        return this.findingFP;
    }

    public void setFindingFP(boolean findingFP) {
        this.findingFP = findingFP;
    }

    public Point getPos() {
        return this.pos;
    }

    public void setPos(int row, int col) {
        this.pos = new Point(col, row);
    }

    public void setPos(Point pos) {
        this.pos = pos;
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

    public boolean isReachedGoal() {
        return this.reachedGoal;
    }

    public void setReachedGoal(boolean reachedGoal) {
        this.reachedGoal = reachedGoal;
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

    public void setSensorRes(HashMap<String, Integer> sensorRes) {
        this.sensorRes = sensorRes;
    }

    public HashMap<String, ObsSurface> getSurfaceTaken() {
        return surfaceTaken;
    }

    /**
     * Create and set sensors in position and add it to robot's sensorMap
     */
    private void initSensors() {
        int row = this.pos.y;
        int col = this.pos.x;

        //Initialize all 6 sensors (5 short range, 1 long range)
        Sensor[] createdSensors = createSensorsStartPos(row, col);

        //TODO: Remove sensorList
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

    //TODO: Delete this and replace with Robot constants
    private void addSensorsToRobot(){
        this.sensorList.add(RobotConstants.SENSOR_ID[0]);
        this.sensorList.add(RobotConstants.SENSOR_ID[1]);
        this.sensorList.add(RobotConstants.SENSOR_ID[2]);
        this.sensorList.add(RobotConstants.SENSOR_ID[3]);
        this.sensorList.add(RobotConstants.SENSOR_ID[4]);
        this.sensorList.add(RobotConstants.SENSOR_ID[5]);
    }

    /**
     * Add sensors to robot
     * @param sensors Sensors to add to robot
     */
    private void createSensorMap(Sensor[] sensors){
        for(Sensor s: sensors){
            sensorMap.put(s.getId(), s);
        }
    }

    /**
     * Update sensor's direction when robot moves
     * @param rowDiff Net displacement in row coordinates
     * @param colDiff Net displacement in col coordinates
     */
    private void setSensorPos(int rowDiff, int colDiff) {
        Sensor s;
        for (int i=0; i< RobotConstants.SENSOR_ID.length; i++) {
            s= sensorMap.get(RobotConstants.SENSOR_ID[i]);
            s.setPos(s.getRow() + rowDiff, s.getCol() + colDiff);
        }
    }

    /**
     * @return True when is real exploration, false otherwise
     */

    private boolean isRealExploration(){
        return (!this.sim);
    }


    /**
     * Update sensor position when robot turns left or right
     * @param s Sensor whose position is to be updated
     * @param turn_dir Direction of the turn
     */
    private void locateSensorAfterRotation(Sensor s, Direction turn_dir){
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

    /**
     * Update all sensors' position when robot turns left or right
     * @param turn_dir Direction of the turn
     */
    private void internalRotateSensor(Direction turn_dir) {
        switch (turn_dir) {
            case LEFT:
                for (int i = 0; i< RobotConstants.NO_OF_SENSORS; i++) {
                    Sensor s = sensorMap.get(RobotConstants.SENSOR_ID[i]);
                    //Change sensor direction
                    s.setSensorDir(Direction.getAntiClockwise(s.getSensorDir()));
                    //Change sensor position
                    locateSensorAfterRotation(s, turn_dir);
                }
                break;
            case RIGHT:
                for (int i = 0; i< RobotConstants.NO_OF_SENSORS; i++) {
                    //Change sensor direction
                    Sensor s = sensorMap.get(RobotConstants.SENSOR_ID[i]);
                    //Change sensor position
                    s.setSensorDir(Direction.getClockwise(s.getSensorDir()));
                    locateSensorAfterRotation(s, turn_dir);
                }
                break;
            default:
                LOGGER.warning("No rotation done. Wrong input direction: " + turn_dir);
        }
    }

    /**
     * Update all sensors' position when robot turns left, right or u-turn (abstraction of internalRotateSensor)
     * @param turn_dir Direction of the turn
     */
    private void rotateSensors(Direction turn_dir) {
        switch (turn_dir) {
            case LEFT:
                internalRotateSensor(Direction.LEFT);
                break;
            case RIGHT:
                internalRotateSensor(Direction.RIGHT);
                break;
            case DOWN:
                internalRotateSensor(Direction.RIGHT);
                internalRotateSensor(Direction.RIGHT);
                break;
            default:
                break;
        }
    }
    /**
     * Updates the alignCount when robot moves (represent how many steps robot moved w/o alignment)
     */
    private void updateAlignCount(int steps){
        alignCount += steps;
    }

    /**
     * Converts the movement command (move forward, move back, etc) to string and sends to Arduino for execution
     * @param cmd Command to be converted
     * @param steps No of steps to be taken in integer form
     */
    private void sendCommandToArduino(Command cmd, int steps){

        String cmdStr = getCommand(cmd, steps);
        NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);

    }

    /**
     * Evaluate multiplier of row increment when robot moves (forward or backward) in the direction it is facing
     * @param cmd Command of either forward or backward movement
     * @return Row multiplier to be used in calculation of robot's new position
     */
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

    /**
     * Evaluate multiplier of column increment when robot moves (forward or backward) in the direction it is facing
     * @param cmd Command of either forward or backward movement
     * @return Column multiplier to be used in calculation of robot's new position
     */
    private int getColIncrementForMovement(Command cmd){
        int colInc = 0;
        //Determines
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
//                printer.setText(printer.getText() + status + "\n");
                LOGGER.warning(status);
        }
        return colInc;

    }
    /**
     * Evaluate multiplier of row increment of sensor when sensing
     * @param dir Direction that sensor is facing
     * @return Row multiplier to be used in evaluation during sensing
     */
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

    /**
     * Evaluate multiplier of column increment of sensor when sensing
     * @param dir Direction that sensor is facing
     * @return Column multiplier to be used in evaluation during sensing
     */
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

    /**
     * Simulate delay in simulated exploration to match the steps per second of the robot (preset variable)
     * @param stepsPerSecond No of steps (forward or backward movement) taken per second
     * @param steps No of steps to be taken
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private void simulateDelay(int stepsPerSecond,int steps) throws InterruptedException{
        if (this.sim) {
            this.tempEndTime = System.currentTimeMillis();
            this.tempDiff = RobotConstants.WAIT_TIME / stepsPerSecond * steps - (this.tempEndTime - this.tempStartTime);
            if (this.tempDiff > 0) {
                TimeUnit.MILLISECONDS.sleep(this.tempDiff);
            }
        }
    }

    /**
     * @return Returns true if robot is executing exploration, false otherwise
     */
    private boolean isExploration(){
        return !(this.findingFP);
    }


    /**
     * Moves robot in either forward or backward direction with the specified number of steps, and updates exploration map
     * @param cmd Command of either forward or backward movement
     * @param steps Number of steps robot is supposed to move
     * @param exploredMap Map of explored part of arena
     * @exception InterruptedException Throws exception when parameter is null
     */

    public void move(Command cmd, int steps, Map exploredMap, int stepsPerSecond) throws InterruptedException {

        this.tempStartTime = System.currentTimeMillis();

        //Send command to Arduino and update calibration variable if robot is executing real exploration
        if (isRealExploration()) {
            sendCommandToArduino(cmd,steps);
            updateAlignCount(steps);
            lastR2Pos = sensorMap.get("R2").getPos();
        }

        //Determines increment of row and column coordinates with each step depending on direction of robot
        int rowInc = getRowIncrementForMovement(cmd);
        int colInc = getColIncrementForMovement(cmd);

        int newRow = pos.y + rowInc * steps;
        int newCol = pos.x + colInc * steps;

        if(exploredMap.checkValidMove(newRow, newCol)) {
            preMove = cmd;
            status = String.format("%s for %d steps\n", cmd.toString(), steps);
            LOGGER.info(status);
            LOGGER.info("row = " + newRow + ", col = " + newCol);

            //Simulate delay to match preset variable of steps per second
            if(sim) {
                simulateDelay(stepsPerSecond, steps);
            }
            //Update robot's position
            this.setPosition(newRow, newCol);

            //Update explored map
            if(isExploration()) {
                for (int i = 0; i < steps; i++) {
                    exploredMap.setPassThru(newRow - rowInc * i, newCol - colInc * i);
                }
            }
        }
    }

    /**
     * Updates robot direction and all sensors' direction when robot turns
     * @param cmd Command of either turn left or turn right
     */

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

    /**
     * Turns robot in either left or right direction, speed adjusted to preset steps per second of robot
     * @param cmd Command of either turn left or right
     * @param stepsPerSecond Number of steps robot is set to take per second
     * @throws InterruptedException If cannot connect to Arduino
     */
    public void turn(Command cmd, int stepsPerSecond) throws InterruptedException {

        tempStartTime = System.currentTimeMillis();
        //Send command to Arduino and update calibration variable if robot is executing real exploration
        if(isRealExploration()){
            sendCommandToArduino(cmd,1);
            updateAlignCount(1);
            lastR2Pos = sensorMap.get("R2").getPos();
        }

        changeRobotAndSensorDirection(cmd);

        preMove = cmd;
        status = cmd.toString() + "\n";
        LOGGER.info(status);
        LOGGER.info(pos.toString());


        //Simulate delay for steps per second only for simulator mode
//        if(sim) {
//            simulateDelay(stepsPerSecond, 1);
//        }
    }

    /**
     * Updates robot's position and cells of starting position on explored map
     * @param row Row that robot is set in
     * @param col Column that robot is set in
     * @param exploredMap Map of explored part of the arena
     */
    public void setStartPos(int row, int col, Map exploredMap) {
        setPosition(row, col);
        exploredMap.setAllExplored(false);
        exploredMap.setAllMoveThru(false);
        for (int r = row - 1; r <= row + 1; r++) {
            for (int c = col - 1; c <= col + 1; c++) {
                exploredMap.getCell(r, c).setExplored(true);
                exploredMap.getCell(r, c).setMoveThru(true);
            }
        }
    }

    /**
     * Updates robot position and sensor's position when robot changes position (due to movement, etc)
     * @param row Row coordinates of robot's updated position
     * @param col Column coordinates of robot's updated position
     */

    private void setPosition(int row, int col) {
        int colDiff = col - this.pos.x;
        int rowDiff = row - this.pos.y;
        this.pos.setLocation(col, row);
        setSensorPos(rowDiff, colDiff);
    }

    /**
     * Display robot's current coordinates and direction and sensor's direction in log
     */

//    private void logSensorInfo() {
//        for (String sname : RobotConstants.SENSOR_ID) {
//            Sensor s = sensorMap.get(sname);
//            String info = String.format("id: %s\trow: %d; col: %d\tdir: %s\n", s.getId(), s.getRow(), s.getCol(), s.getSensorDir());
//            LOGGER.info(info);
//        }
//    }

    /**
     * Parse start point sent from Android
     * @param jsonMsg Message containing start point
//     * @return Coordinates of start point
//     */
    public Point parseStartPointJson(String jsonMsg) {
        if (jsonMsg.contains(NetworkConstants.START_POINT_KEY)) {
            JSONObject startPointJson = new JSONObject(new JSONTokener(jsonMsg));
            //TODO: Might need adjustment based on message format
            return (new Point((int) startPointJson.get("x") , (int) startPointJson.get("y")));
        }
        else {
            LOGGER.warning("Not a start point msg. Return null.");
            return null;
        }
    }

    /**
     * Parse way point sent from Android
     * @param jsonMsg Message containing way point
     * @return Coordinates of waypoint
     */
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

    /**
     * Obtain sensor result from Arduino (through Rasberry Pi) and updates sensor result in robot
     * @return HashMap<SensorId, ObsBlockDis>
     */

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

    /**
     * Evaluate multiplier of row increment of camera during image recognition
     * @return Returns row multiplier
     */

//    private int getRowIncrementForCamera(){
//        int rowInc=0;
//        switch (dir) {
//            case LEFT:
//                rowInc = 1;
//                break;
//            case RIGHT:
//                rowInc = -1;
//                break;
//        }
//        return rowInc;
//    }

    /**
     * Evaluate multiplier of column increment of camera during image recognition
     * @return Returns col multiplier
     */
//    private int getColIncrementForCamera(){
//        int colInc=0;
//        switch (dir) {
//            case UP:
//                colInc = 1;
//                break;
//            case DOWN:
//                colInc = -1;
//                break;
//        }
//        return colInc;
//
//    }

    /**
     * Determine whether image is detected on right side of robot for image recognition
     * @return True if image is detected
     */

//    private boolean checkIfRightImageDetected(){
//        if ((sensorRes.get("R1") > 0 && sensorRes.get("R1") <= RobotConstants.CAMERA_MAX)
//                || (sensorRes.get("R2") > 0 && sensorRes.get("R2") <= RobotConstants.CAMERA_MAX)) {
//            return !isRightHuggingWall();
//        }
//        return false;
//
//    }


    /**
     * Detects if there are possible image surfaces on the right of the robot that have not been detected before
     * Signals RPi for image recognition if so, and updates memory of captured surfaces
     * @param exploredMap Map of explored part of the arena
     * @return Array of obstacle surfaces that have been taken (might or might not have images on them)
     */
//    public ArrayList<ObsSurface> imageRecognitionRight(Map exploredMap) {
//
//        int rowInc, colInc;
//        int camera_row, camera_col, temp_row, temp_col;
//        int camInc;
//
//        rowInc = getRowIncrementForCamera();
//        colInc = getColIncrementForCamera();
//        camera_row = this.pos.y + rowInc;
//        camera_col = this.pos.x + colInc;
//
//        boolean sendRPI, hasObsAtCamAxis = false;
//
//        //Right sensor to check if image is detected; send RPI command for image recognition if detected
//        sendRPI = checkIfRightImageDetected();
//
//        //Check for all values within camera detection range
//        for (camInc = RobotConstants.CAMERA_MIN; camInc <= RobotConstants.CAMERA_MAX; camInc++) {
//            temp_row = camera_row + rowInc * camInc;
//            temp_col = camera_col + colInc * camInc;
//
//            //If obstacle is obstacle and explored , do image recognition
//            if (exploredMap.checkValidCell(temp_row, temp_col)) {
//                Cell temp_cell = exploredMap.getCell(temp_row, temp_col);
//                //Check if cell has been explored and is obstacle
//                if (temp_cell.isExplored() && temp_cell.isObstacle()) {
//                    // Images can only be on obstacle surfaces; send RPi signal for image recognition
//                    sendRPI = true;
//                    hasObsAtCamAxis = true;
//                    break;
//                }
//            } else {
//                break;
//            }
//        }
//
//        //TODO: Not sure what is this
//        if (!hasTurnAndAlign && (preMove == Command.TURN_LEFT || preMove == Command.TURN_RIGHT)) {
//            imageCount = 0;
//        }
//
//        //Initialize array of surfaces to return
//        ArrayList<ObsSurface> surfaceTakenList = new ArrayList<ObsSurface>();
//        ObsSurface tempObsSurface;
//
//        if (sendRPI) {
//            if (imageCount == 0) {
//                //TODO: Check again what is the character for Algo to RPi
//                //Send position of camera and intended direction of camera
//                String to_send = String.format("I%d|%d|%s", camera_col + 1, camera_row + 1, Direction.getClockwise(dir).toString());
//                //Check if image has already been detected
//                if (!imageHashSet.contains(to_send)) {
//                    //Append image to sent image
//                    imageHashSet.add(to_send);
//                    NetMgr.getInstance().send(to_send);
//
//                    //Add surface to robot memory and return surface object if sensor detects valid surface
//                    tempObsSurface = addToSurfaceTaken("R1", rowInc, colInc);
//                    if (tempObsSurface != null) {
//                        surfaceTakenList.add(tempObsSurface);
//                    }
//
//                    tempObsSurface = addToSurfaceTaken("R2", rowInc, colInc);
//                    if (tempObsSurface != null) {
//                        surfaceTakenList.add(tempObsSurface);
//                    }
//
//                    //Adds to surface list if sensor previously detected obstacle at axis of camera; will return if its valid
//                    if (hasObsAtCamAxis) {
//                        tempObsSurface = internalAddToSurfaceTaken(camera_row, camera_col, rowInc, colInc, camInc);
//                        if (tempObsSurface != null) {
//                            surfaceTakenList.add(tempObsSurface);
//                        }
//                    }
//
//                }
//            }
//            //TODO: Not sure what this is for
//            imageCount = (imageCount + 1) % 2;
//
//        }
//        else {
//            imageCount = 0;
//        }
//        LOGGER.info(Boolean.toString(sendRPI));
//        LOGGER.info(String.format("imageCount: %d", imageCount));
//        return surfaceTakenList;
//    }

    /**
     * Checks if obstacle surface detected by sensor is a valid reading; appends to array of surfaces taken if true
     * @param sensorName Sensor ID
     * @param rowInc Row increment of surface position from camera
     * @param colInc Col increment of surface position from camera
     * @return Object of ObstacleSurface if the reading is valid, null otherwise
     */

//    private ObsSurface addToSurfaceTaken(String sensorName, int rowInc, int colInc) {
//        int tempSensorRow, tempSensorCol, tempSensorReading;
//
//        tempSensorReading = sensorRes.get(sensorName);
//
//        //Check if detected result is valid
//        if (tempSensorReading > 0 && tempSensorReading <= RobotConstants.CAMERA_MAX) {
//            tempSensorRow = sensorMap.get(sensorName).getRow();
//            tempSensorCol = sensorMap.get(sensorName).getCol();
//            return internalAddToSurfaceTaken(tempSensorRow, tempSensorCol, rowInc, colInc, tempSensorReading);
//        }
//        else {
//            return null;
//        }
//
//    }

    /**
     * Append surfaces captured by RPi to the robot's memory
     * @param tempRow Row coordinates of camera
     * @param tempCol Column coordinates of camera
     * @param rowInc Row increment multiplier (dependent on robot direction)
     * @param colInc Column increment multiplier (dependent on robot direction)
     * @param incStep Distance from camera
     * @return Obstacle surface that is captured
     */

//    private ObsSurface internalAddToSurfaceTaken(int tempRow, int tempCol, int rowInc, int colInc, int incStep) {
//        int tempObsRow, tempObsCol;
//        ArrayList<ObsSurface> surfaceTakenList = new ArrayList<ObsSurface>();
//        ObsSurface tempObsSurface;
//        Direction tempSurface;
//        tempObsRow = tempRow + rowInc * incStep;
//        tempObsCol = tempCol + colInc * incStep;
//        tempSurface = Direction.getAntiClockwise(dir);
//        tempObsSurface = new ObsSurface(tempObsRow, tempObsCol, tempSurface);
//        surfaceTaken.put(tempObsSurface.toString(), tempObsSurface);
//        return tempObsSurface;
//    }

    /** TODO
     * Check whether image recognition is possible (front obstacles )
     * i.e. obstacles found 2 grids in front of any front sensors
     * if yes, send to RPI
     * format: I|X|Y|RobotDirection
     */
    //TODO: REMOVE IF NOT USED
//    public void imageRecognitionFront() {
//        if (sensorRes.get("F1") == 2 || sensorRes.get("F2") == 2 || sensorRes.get("F3") == 2) {
//            // TODO: check using android index or algo index
//            Sensor F2 = sensorMap.get("F2");
//            String to_send = String.format("I%d|%d|%s", F2.getCol() + 1, F2.getRow() + 1, dir.toString());
//            NetMgr.getInstance().send(to_send);
//        }
//    }


    /**
     * Obtain sensor result from realMap in simulation mode (no real sensor needed)
     * @param realMap Map of entire arena with obstacles
     * @return HashMap<SensorId, ObsBlockDis> Array of sensor and their corresponding detected result
     */

    public HashMap<String, Integer> updateSensorRes(Map realMap) {
        int obsBlock;
        for(String sname: RobotConstants.SENSOR_ID) {

            obsBlock = sensorMap.get(sname).detect(realMap);
            sensorRes.put(sname, obsBlock);
        }
        return sensorRes;
    }
    public void setR1count(int R1count){
        this.R1count = R1count;
    }

    public boolean isSamePoint(Point p1, Point p2){
        if((p1.x == p2.x) && (p1.y == p2.y)){
            return true;
        }
        return false;
    }

    /**
     * Sensing surrounding cells for obstacles in simulation mode
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the arena
     * @return Array of surface of obstacles
     */

    public ArrayList<ObsSurface> sense(Map exploredMap, Map realMap) {
        HashMap<String, Integer> sensorResult = completeUpdateSensorResult(realMap);
        updateMap(exploredMap, sensorResult);



        if (isRealExploration()) {
            //Send updated map to Android
            send_android(exploredMap);

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

    /**
     * Update sensorRes but not the map. No alignment as well. Send image as well.
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the arena
     */

    public void senseWithoutMapUpdateAndAlignment(Map exploredMap, Map realMap) {

        completeUpdateSensorResult(realMap);

        if (isRealExploration()) {
            send_android(exploredMap);
        }

//        if (isRealExploration()) {
//            //Send updated map to Android
//            send_android(exploredMap);
//
//            //If robot is adjacent to right wall, will use normal calibrate after steps
//            if(isRightHuggingWall() && alignCount> RobotConstants.CALIBRATE_AFTER){
//                align_right(exploredMap, realMap);
//                R1count = 0;
//                alignCount = 0;
//            }
//            else if(R1count == 3){
//                align_right(exploredMap, realMap);
//            }
//
//        }




    }


    /**
     * Update sensorRes but does not update the map nor calibrate (for image recognition, not exploration)
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the arena
     */
        public ArrayList<ObsSurface> senseWithoutMapUpdate(Map exploredMap, Map realMap)  {

        completeUpdateSensorResult(realMap);

        //For real exploration, send exploredMap to android
        if (isRealExploration()) {
            send_android(exploredMap);

            //If robot is adjacent to right wall, will use normal calibrate after steps
            if (isRightHuggingWall() && alignCount > RobotConstants.CALIBRATE_AFTER) {
                align_right(exploredMap, realMap);
                R1count = 0;
                alignCount = 0;
            } else if (R1count == 3) {
                align_right(exploredMap, realMap);
            }
        }
        return null;
    }

    /**
     * Turn right and calibrate front sensors to align with obstacle/wall; turn left and calibrate right sensors to
     * align with obstacle/wall
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the arena
     * @throws InterruptedException If cannot sense
     */
    public void turnRightAndAlignMethod(Map exploredMap, Map realMap) throws InterruptedException {

        if((sensorRes.get("F1") == 1 && sensorRes.get("F3") == 1)) {
            //TODO: Change order if its wrong
            senseWithoutAlign(exploredMap, realMap);
            align_front(exploredMap, realMap);
        }

        senseWithoutAlign(exploredMap, realMap);
        align_right(exploredMap, realMap);

        hasTurnAndAlign = true;
        turnAndAlignCount = 0;
    }

    /**
     * Turn right and calibrate front sensors to align with obstacle/wall; turn left and calibrate right sensors to
     * align with obstacle/wall, without updating of map
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the arena
     * @throws InterruptedException If cannot sense
     */
    public void turnRightAndAlignMethodWithoutMapUpdate(Map exploredMap, Map realMap) throws InterruptedException {

        senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
        align_right(exploredMap, realMap);

        senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
        align_front(exploredMap, realMap);

        hasTurnAndAlign = true;
        turnAndAlignCount = 0;
    }

    /**
     * Robot sensing surrounding obstacles in simulator mode
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map obstacles of the entire arena
     */
    private void senseWithoutAlign(Map exploredMap, Map realMap) {
        HashMap<String, Integer> sensorResult = completeUpdateSensorResult(realMap);
        updateMap(exploredMap, sensorResult);

        // Send updated map to android
        if (isRealExploration()) {
            send_android(exploredMap);
        }
    }

    /**
     * Updates results from sensor in both simulated and real exploration
     * @param realMap Map of Map of entire arena with obstacles
     * @return HashMap of sensor result
     */
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

    /**
     * Obtain row increment for robot's position for every direction
     * @param dir Direction of movement
     * @return Row coordinates after movement
     */
    private int getRowIncrementForMovement(Direction dir) {
        int rowInc = 0;

        switch (dir) {
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
    /**
     * Obtain column increment for robot's position for every direction
     * @param dir Direction of movement
     * @return Column coordinates after movement
     */
    private int getColIncrementForMovement(Direction dir) {
        int colInc = 0;

        switch (dir) {
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
//Obstaclesurfaces here
    public void removeObstacleSurfaces(Point point){
        if(!imageRec) {
            for (int i = 0; i < obsSurfaces.size(); i++) {
                if (isSamePoint(obsSurfaces.get(i).getPos(), point)) {
                    obsSurfaces.remove(obsSurfaces.get(i));
                }
            }
        }
    }

    public boolean obstacleSurfaceExists(ObsSurface obsSurface){
        for (int i = 0; i < obsSurfaces.size(); i++) {
            if ((obsSurface.getPos() == obsSurfaces.get(i).getPos()) && obsSurface.getSurface() == obsSurfaces.get(i).getSurface()) {
                return true;
            }
        }
        return false;
    }
    public void addObstacleSurface(Point obstaclePos, Direction sensorDir) {

//        if(!imageRec) {
//        Direction obstacleDir = Direction.getOpposite(sensorDir);
//        int rowInc, colInc, tempRow,tempCol;
//        Cell tempCell = new Cell(obstaclePos);
//        Cell tempCell2;
//        //For each direction, get the increment for row/c
//        //Consider all possible surfaces of obstacle
//        rowInc = getRowIncrementForMovement(obstacleDir);
//        colInc = getColIncrementForMovement(obstacleDir);
//        for (int l = 1; l <= RobotConstants.CAMERA_RANGE; l++) {
//            tempRow = tempCell.getPos().y + rowInc * l;
//            tempCol = tempCell.getPos().x + colInc * l;
//            if (exploredMap.checkValidCell(tempRow, tempCol)) {
//                tempCell2 = exploredMap.getCell(tempRow, tempCol);
//                if (tempCell2.isObstacle()) {
//                    break;
//                }
//                else {
//                    if (l == RobotConstants.CAMERA_RANGE) {
//                        if (!tempCell2.isVirtualWall()) {
//                            ObsSurface obsSurface = new ObsSurface(tempCell.getPos(), tempCell2.getPos(), obstacleDir, sensorDir);
//                            obsSurfaces.add(obsSurface);
//                            System.out.println("Created obstacle surface");
//                        }
//                        else {
//                            ArrayList<Cell> possibleNeighbours = exploredMap.getNeighbours(tempCell, dir);
//                            System.out.print("Neighbours of Cell: "+ tempCell.getPos().x + " , " + tempCell.getPos().y + "\n");
//                            System.out.print(possibleNeighbours.size());
//                            for (int m = 0; m < possibleNeighbours.size(); m++) {
//
//                                Cell neighbourCell = possibleNeighbours.get(m);
//                                for (int n = 1; n <= RobotConstants.CAMERA_RANGE; n++) {
//                                    tempRow = neighbourCell.getPos().y + rowInc * n;
//                                    tempCol = neighbourCell.getPos().x + colInc * n;
//                                    if (exploredMap.checkValidCell(tempRow, tempCol)) {
//                                        tempCell2 = exploredMap.getCell(tempRow, tempCol);
//                                        if (tempCell2.isObstacle()) {
//                                            break;
//                                        } else {
//                                            if (n == RobotConstants.CAMERA_RANGE) {
//                                                if (!tempCell2.isVirtualWall()) {
//                                                    System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is NOT obstacle OR virtual wall");
//
//                                                    ObsSurface obsSurface = new ObsSurface(tempCell.getPos(), tempCell2.getPos(), dir, Direction.getOpposite(dir));
//                                                    obsSurfaces.add(obsSurface);
//                                                    System.out.println("Created obstacle surface");
//                                                }
//                                            }
//                                        }
//                                    } else {
//                                        break;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }else {
//                System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is INVALID");
//                break;
//            }
//        }
//    }
}
//    }
//}
//    }
//
//
//            if(!obstacleSurfaceExists(obsSurface)){
//                obsSurfaces.add(obsSurface);
//            }
//            System.out.println("Created obstacle surface");
//        }
//    }

    /**
     * Update the map with result obtained from sensor
     * @param exploredMap Map of explored part of the arena
     * @param sensorResult Hashmap of the sensor result obtained from sensors
     */
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
                                            removeObstacleSurfaces(new Point(tempRow, tempCol));
                                        }
                                        exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), false);
                                    } else {
                                        break;
                                    }

                                }
                                exploredMap.reinitVirtualWall();
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
                                        addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                                    }
                                    for (int i = 1; i < obsBlock; i++) {
                                        tempRow1 = s.getRow() + rowInc * i;
                                        tempCol1 = s.getCol() + colInc * i;
                                        exploredMap.getCell(tempRow1, tempCol1).setExplored(true);
                                        exploredMap.getCell(tempRow1, tempCol1).setObstacle(false);
                                        if(!imageRec) {
                                            removeObstacleSurfaces(new Point(tempRow, tempCol));
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
                                            addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                                        }
                                        for (int i = 1; i < obsBlock; i++) {
                                            tempRow2 = s.getRow() + rowInc * i;
                                            tempCol2 = s.getCol() + colInc * i;
                                            exploredMap.getCell(tempRow2, tempCol2).setExplored(true);
                                            exploredMap.getCell(tempRow2, tempCol2).setObstacle(false);
                                            if(!imageRec) {
                                                removeObstacleSurfaces(new Point(tempRow, tempCol));
                                            }
                                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow2, tempCol2), false);
                                        }
                                    }
                                }
                                exploredMap.reinitVirtualWall();
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
                                addObstacleSurface(new Point(tempRow, tempCol), s.getSensorDir());
                            }
                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), true);
                            exploredMap.reinitVirtualWall();
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
                                removeObstacleSurfaces(new Point(tempRow, tempCol));
                            }
                            exploredMap.setVirtualWall(exploredMap.getCell(tempRow, tempCol), false);
                            exploredMap.reinitVirtualWall();
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

    /**
     * Translate direction and position of robot into JSON for transmission
     * @return JSONArray of robot's information
     */
    private JSONArray getRobotArray() {

        JSONArray robotArray = new JSONArray();
        JSONObject robotJson = new JSONObject()
                .put("x", pos.x )
                .put("y", pos.y)
                .put("direction", dir.toString().toLowerCase());
        robotArray.put(robotJson);
        return robotArray;
    }


    //TODO: Image Rec
    private JSONArray getImageArray() {

        JSONArray robotArray = new JSONArray();
        JSONObject robotJson = new JSONObject()
                .put("x", pos.x )
                .put("y", pos.y)
                .put("direction", dir.toString().toLowerCase());
        robotArray.put(robotJson);
        return robotArray;
    }


    /**
     * Translate map into JSON array for transmission
     * @param exploredMap Map of explored part of the arena
     * @return JSONArray of map information
     */
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

    /**
     * Translate status of robot into JSONArray
     * @return JSONArray of robot's latest operation
     */
    private JSONArray getStatusArray() {
        JSONArray statusArray = new JSONArray();
        JSONObject statusJson = new JSONObject()
                .put("status", status.replaceAll("\\n",""));
        statusArray.put(statusJson);
        return statusArray;
    }

    /**
     * Send robot's direction and postion, current explored environment of arena to Android
     * @param exploredMap Map of explored part of the arena
     */
    public void send_android(Map exploredMap) {
        JSONObject androidJson = new JSONObject();

        androidJson.put("robot", getRobotArray());
        androidJson.put("map", getMapArray(exploredMap));
        androidJson.put("status", getStatusArray());
        androidJson.put("image", getImageResult());
        //Might be very long
        NetMgr.getInstance().send(NetworkConstants.ANDROID + androidJson.toString() + "\n");

    }

    public String takeImg() {
        System.out.println("Send RPI taking image");

        NetMgr.getInstance().send(NetworkConstants.RPI_TAKEIMG);
        String msg = NetMgr.getInstance().receive();
        while(msg == null){
            msg = NetMgr.getInstance().receive();
        }
        return msg;

    }

//    private boolean backRightCellisObstacleOrWall(Map exploredMap){
//        int rowDiff=0, colDiff=0;
//        switch(robot.getDir()){
//            case UP: {
//                //colDiff is x
//                //rowDiff is y
//                rowDiff = -1;
//                colDiff = 2;
//                break;
//            }
//            case RIGHT:{
//                rowDiff = -2;
//                colDiff = -1;
//                break;
//            }
//            case LEFT:{
//                rowDiff = 2;
//                colDiff = 1;
//                break;
//            }
//            case DOWN:{
//                rowDiff = 1;
//                colDiff = -2;
//                break;
//            }
//        }
//        if(exploredMap.checkValidCell((robot.getPos().x+colDiff),(robot.getPos().y+rowDiff))){
//            return exploredMap.getCell((robot.getPos().x+colDiff),(robot.getPos().y+rowDiff)).isObstacle();
//        }
//        return robot.isRightHuggingWall();
//    }

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
        else if(exploredMap.checkValidCell(F1.x, F1.y) && exploredMap.checkValidCell(F3.x,F3.y)){
            return exploredMap.getCell(F1.x, F1.y).isObstacle() && exploredMap.getCell(F3.x, F3.y).isObstacle();
        }
        return false;
    }
    /**
     * Calibrate robot's direction using front sensors, returns true if obstacles are detected
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map of obstacles in arena
     */
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
                align_front1(exploredMap, realMap);
            }

            status = "Aligning Front 1\n";
            LOGGER.info(status);
            return true;
        }
        return false;
    }
    /**
     * Calibrate robot's direction using one of front sensors
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map of obstacles in arena
     */
    public void align_front1(Map exploredMap, Map realMap) {
        //Robot directly in front of obstacle/wall
            // Send align front command to Arduino
            String cmdStr = getCommand(Command.ALIGN_FRONT1, 1);  // steps set to 0 to avoid appending to cmd
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            senseWithoutAlign(exploredMap, realMap);
            turnAndAlignCount = 0;
        }

    /**
     * Calibrate robot's direction using front sensors
     */
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
    /**
     * Calibrate robot's direction using right sensors
     * @param exploredMap Map of explored part of the arena
     * @param realMap Map of obstacles in arena
     */
    public void align_right(Map exploredMap, Map realMap) {
        //Arduino will double check again if can align
//        if (sensorRes.get("R1") == 1) {
        if(checkRightisObstacleOrWall(exploredMap)) {
            String cmdStr = getCommand(Command.ALIGN_RIGHT, 1);
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            alignCount = 0;
            status = String.format("Aligning Right: %d\n", 1);
            LOGGER.info(status);

            senseWithoutAlign(exploredMap, realMap);
//        }
        }
    }
    public void align_right1(Map exploredMap, Map realMap) {
        //Arduino will double check again if can align
//        if (sensorRes.get("R1") == 1) {

            String cmdStr = getCommand(Command.ALIGN_RIGHT, 1);
            NetMgr.getInstance().send(NetworkConstants.ARDUINO + cmdStr);
            alignCount = 0;
            status = String.format("Aligning Right: %d\n", 1);
            LOGGER.info(status);

            senseWithoutAlign(exploredMap, realMap);
//        }

    }
    /**
     * Robot is right hugging the wall if the right sensor position is equal to
     * the lowest or highest possible row or col number
     * @return True if right hugging the wall; false otherwise
     */
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
    /**
     * Robot is right hugging the wall if the right sensor position is equal to
     * the lowest or highest possible row or col number
     * @return True if right hugging the wall; false otherwise
     */
    public boolean isFacingWall() {
        Point R1_pos = sensorMap.get("R1").getPos();
        Point R2_pos = sensorMap.get("R2").getPos();
        if (R1_pos.y == 0 && R2_pos.y ==0 ){
            return true;
        }
        if(R1_pos.x == MapConstants.MAP_WIDTH - 1 && R2_pos.x == MapConstants.MAP_WIDTH - 1){
            return true;
        }
        if(R1_pos.x == 0 && R2_pos.x == 0){
            return true;
        }
        return (R1_pos.y == MapConstants.MAP_HEIGHT-1 && R2_pos.y == MapConstants.MAP_HEIGHT-1);
    }




    public int getAlignCount() {
        return alignCount;
    }

    public void setAlignCount(int alignCount) {
        this.alignCount = alignCount;
    }

    public int getTurnAndAlignCount() {
        return turnAndAlignCount;
    }

    public void setTurnAndAlignCount(int counter) {
        this.turnAndAlignCount = counter;
    }

    public boolean getHasTurnAndAlign() {
        return hasTurnAndAlign;
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

    /**
     * Convert the command to a string format for sending to RPi/Arduino
     * @param cmd Command of move/turn
     * @param steps No of steps to move in that direction
     * @return String format of the command for transmission
     */
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
            System.out.println("ERROR IF ANY");
            File path = new File("Y:/ImageRec/");
            Process p = Runtime.getRuntime().exec(cmdArray, null, path);

            BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String result;
            p.waitFor();

            while (true) {
                System.out.println("Attempting to get image result from Python Script");
                if((result = in.readLine()) != null){
                    System.out.println("Yo yo");
                    System.out.println(result);
                    in.close();
                    p.destroy();
                    return result;
                }
                System.out.println("Problem?");
            }
        }
        catch(Exception e){
            System.out.println(e);
        }
        return msg;
    }





    // TODO: Remove when done
    public static void main(String[] args) throws InterruptedException{
        Robot robot = new Robot(true, true,1, 1, Direction.UP);
        System.out.println(robot.status);
        Map exploredMap = new Map();

        robot.send_android(exploredMap);


    }

}


