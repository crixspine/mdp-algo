package Algorithm;

import Map.Map;
import Map.Cell;
import Map.Direction;
import Map.MapConstants;
import Map.ObsSurface;

import Network.NetMgr;
import Network.NetworkConstants;
import Robot.Robot;
import Robot.Command;
import Robot.RobotConstants;
import org.json.JSONObject;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

public class Exploration {

    /**
     * logger to print log information on robot status, movement, position, etc.
     * ExploredMap to hold current explored environment
     * RealMap to hold entire environment of arfena (obstacles, free cells, etc)
     * CoverageLimit to reflect no of cells to explore before ceasing exploration
     * TimeLimit to reflect the time limit before ceasing exploration
     * StepPerSecond to reflect the preset no of steps per second of the robot
     * Sim to show simulation mode (true) or real run (false)
     * AreaExplored to reflect no of cells explored
     * StartTime, EndTime are variables used enforce time limit
     * Start is start position for robot in x,y coordinates
     * NotYetTaken to track obstacle surfaces that are yet to be taken
     */

    private static final Logger LOGGER = Logger.getLogger(Exploration.class.getName());

    private Map exploredMap;

    private Map realMap;
    private Robot robot;
    private double coverageLimit;
    private int timeLimit;
    private int stepPerSecond;
    private boolean sim;
    private double areaExplored;
    private long startTime;
    private long endTime;
    private Point start;
    ArrayList<ObsSurface> obsSurfaces = new ArrayList<ObsSurface>();

    public ArrayList<String> getImageResult() {
        return imageResult;
    }

    public void setImageResult(ArrayList<String> imageResult) {
        this.imageResult = imageResult;
    }

    ArrayList<String> imageResult = new ArrayList<String>();


    private HashMap<String, ObsSurface> notYetTaken;

    // Checking for four consecutive right + forward move

    private int right_move = 0;

    public Exploration(Map exploredMap, Map realMap, Robot robot, double coverageLimit, int timeLimit, int stepPerSecond,
                       boolean sim) {
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.robot = robot;
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit;
        this.stepPerSecond = stepPerSecond;
        this.sim = sim;
    }

    //Getters and setters
    public Map getExploredMap() {
        return exploredMap;
    }

    public void setExploredMap(Map exploredMap) {
        this.exploredMap = exploredMap;
    }

    public double getCoverageLimit() {
        return coverageLimit;
    }

    public void setCoverageLimit(double coverageLimit) {
        this.coverageLimit = coverageLimit;
    }

    public int getTimeLimit() {
        return timeLimit;
    }

    public void setTimeLimit(int timeLimit) {
        this.timeLimit = timeLimit;
    }

    /**
     * Obtain row increment for robot's position for every direction
     *
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
     *
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

    /**
     * Execute image recognition exploration
     * @param start Coordinates of start point
     * @throws InterruptedException Will throw exception if parameter is null
     */

//    public void imageExploration(Point start) throws InterruptedException {
//        long imageStartTime = System.currentTimeMillis();
//        //Time taken for right wall hug execution
//        int exp_timing = explorationAllRightWallHug(start);
//
//        //Cease image recognition exploration upon return to start point if timing is better than previous record
//        if (exp_timing >= RobotConstants.BEST_EXP_TIMING) {
//        //If exploration timing not the best timing, begin image recognition
//            //Initialize HashMap of all possible obstacle surfaces
//            robot.setDoingImage(true);
//            notYetTaken = getUntakenSurfaces();
//
//            //If all surfaces taken; cease function
//            if (notYetTaken.size() == 0) {
//                return;
//            }
//            // Calibrate robot at start point after exploration complete
//            calibrate_at_start_before_going_out();
//            System.out.println("DEBUG " + notYetTaken);
//            // Repeatedly move robot to nearest obstacle surface that is not yet captured
//            while (notYetTaken.size() > 0) {
//                imageLoop();
//            }
//            // Upon capturing all possible obstacle surfaces, return to start point
//            goToPoint(start);
//        }
//
//    }


    /**
     * Moves robot to nearest obstacle surface and start right wall hugging algorithm for image recognition
     * @throws InterruptedException Will throw exception if parameter is null
     */
//    private void imageLoop() throws InterruptedException {
//        boolean doingImage = true;
//        ArrayList<ObsSurface> surfTaken;
//        ObsSurface nearestObstacle;
//        Cell nearestCell;
//        boolean success;
//        LOGGER.info("image Loop");
//
//        //Find nearest obstacle surface that has not been captured
//        nearestObstacle = exploredMap.nearestObsSurface(robot.getPos(), notYetTaken);
//
//        System.out.println("DEBUG nearestObstacle " + nearestObstacle.toString());
//
//        //Find nearest cell to specified obstacle that has a movable path to
//        nearestCell = exploredMap.nearestMovable(nearestObstacle);
//        System.out.println("DEBUG nearestCell is null:" + (nearestCell == null));
//
//
//        if (nearestCell != null) {
//            System.out.println("DEBUG nearestCell " + nearestCell.toString());
//
//            //Move robot to nearest cell
//            success = goToPointForImage(nearestCell.getPos(), nearestObstacle);
//            //If move to nearest cell successfully
//            if (success) {
//                System.out.println("DEBUG cell pos " + nearestCell.getPos().toString());
//                do {
//                    robot.setImageCount(0);
//                    //Capture surface of obstacle using right sensors
//                    surfTaken = robot.imageRecognitionRight(exploredMap);
//                    //Remove Obstacle Surface from HashMap of surfaces not taken
//                    updateNotYetTaken(surfTaken);
//                    //Continue moving in right-wall-hug method from obstacle
//                    rightWallHug(doingImage);
//                    System.out.println("DEBUG robot pos " + robot.getPos().toString());
//                    //Robot has not reached nearestCell nor initiated rightHuggingWall
//                } while (!robot.getPos().equals(nearestCell.getPos()) && !robot.isRightHuggingWall());
//            }
//            else {
//                System.out.println("DEBUG in inner else");
//                //Nearest obstacle cannot be reached
//                removeFromNotYetTaken(nearestObstacle);
//            }
//
//        }
//        else {
//            //Cannot compute nearest obstacle
//            System.out.println("DEBUG in outer else");
//            removeFromNotYetTaken(nearestObstacle);
//            System.out.println("DEBUG after removing in outer else");
//        }
//    }

    /**
     * Remove captured surfaces from HashMap of obstacle surfaces that has not been taken
     * @param surfTaken Array of Obstacle surface that has been captured
     */
//    private void updateNotYetTaken(ArrayList<ObsSurface> surfTaken) {
//        for (ObsSurface obsSurface : surfTaken) {
//            if (notYetTaken.containsKey(obsSurface.toString())) {
//                notYetTaken.remove(obsSurface.toString());
//                LOGGER.info("Remove from not yet taken: " + obsSurface);
//            }
//        }
//    }

    /**
     * Remove captured surface from HashMap of obstacle surfaces that has not been taken
     * @param obsSurface Obstacle surface that has been captured
     */
//    private void removeFromNotYetTaken(ObsSurface obsSurface) {
//        notYetTaken.remove(obsSurface.toString());
//        LOGGER.info("Remove from not yet taken: " + obsSurface.toString());
//
//    }

    /**
     *
     * @param loc Coordinates of target point
     * @param obsSurface Target surface
     * @return True if successfully reach the point, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
//    private boolean goToPointForImage(Point loc, ObsSurface obsSurface) throws InterruptedException {
//        robot.setStatus("Go to point: " + loc.toString());
//        LOGGER.info(robot.getStatus());
//        ArrayList<Command> commands;
//        ArrayList<Cell> path;
//        FastestPath fp = new FastestPath(exploredMap, robot, sim);
//        //Obtain path from robot's position to target location
//        path = fp.runAStar(robot.getPos(), loc, robot.getDir());
//        if (path == null) {
//            return false;
//        }
//
//        //Convert fastest path to commands
//        fp.displayFastestPath(path, true);
//        commands = fp.getPathCommands(path);
//        System.out.println("Exploration Fastest Commands: "+commands);
//
//        executeCommandsMoveToTarget(commands,loc, obsSurface);
//
//        // Re- orientate the robot to ensure right side is facing obstacle
//        Direction desiredDir = Direction.getClockwise(obsSurface.getSurface());
//        turnRobotToFaceDirectionDuringImage(desiredDir);
//
//        return true;
//    }

    /**
     * Remove captured surfaces from HashMap of all possible obstacle surfaces
     * @return HashMap of remaining untaken surfaces
     */
//    private HashMap<String, ObsSurface> getUntakenSurfaces() {
//        HashMap<String, ObsSurface> notYetTaken;
//
//        // Obtain all obstacle surfaces after exploration
//        notYetTaken = getAllObsSurfaces();
//        // Remove taken surfaces from HashMap of all possible obstacle surfaces
//        for (String tempObsSurfaceStr : robot.getSurfaceTaken().keySet()) {
//            if (!notYetTaken.containsKey(tempObsSurfaceStr)) {
//                LOGGER.warning("Surface taken not in all possible surfaces. Please check. \n\n\n");
//            }
//            else {
//
//                notYetTaken.remove(tempObsSurfaceStr);
//            }
//        }
//
//        return notYetTaken;
//    }

    /**
     * Check all valid cells for obstacle surface; add them into HashMap and return HashMap upon completion
     * @return HashMap of ObstacleSurfaces
     */
//    private HashMap<String, ObsSurface> getAllObsSurfaces() {
//        // TODO
//        Cell tempCell;
//        Cell temp;
//        ObsSurface tempObsSurface;
//        HashMap<Direction, Cell> tempNeighbours;
//        HashMap<String, ObsSurface> allPossibleSurfaces = new HashMap<String, ObsSurface>();
//        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
//            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
//                tempCell = exploredMap.getCell(row, col);
//
//                if (tempCell.isObstacle()) {
//                    //Get neighbouring cell of specified cell
//                    tempNeighbours = exploredMap.getNeighboursMap(tempCell);
//
//                    //Get all direction of neighbour
//                    for (Direction neighbourDir: tempNeighbours.keySet()) {
//                        temp = tempNeighbours.get(neighbourDir);
//
//                        if (!temp.isObstacle()) {
//                            //Identify obstacle surface and direction
//                            tempObsSurface = new ObsSurface(tempCell.getPos(), neighbourDir);
//                            //Add surface to ArrayList
//                            allPossibleSurfaces.put(tempObsSurface.toString(), tempObsSurface);
//                        }
//                    }
//                }
//
//            }
//        }
//        System.out.println();
//        return allPossibleSurfaces;
//    }

    /**
     * Calibrate robot at start point
     *
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private void calibrate_at_start_before_going_out() throws InterruptedException {
        //Send command to Arduino for initial calibration
        String calibrationCmd = robot.getCommand(Command.INITIAL_CALIBRATE, 1);
        NetMgr.getInstance().send(NetworkConstants.ARDUINO + calibrationCmd);

        //robot.setFindingFP(true);
        robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
        robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
        //robot.setFindingFP(false);
    }


    /**
     * Execute right wall hug and failsafe protocol of moving robot to start point if right wall hug yield no
     * progress during execution.
     * Initiate preparation for second run upon reaching back to start point
     * @param start Coordinates of starting position
     * @return Time taken for completion or execution (if unable to complete right wall hug) in seconds
     * @throws InterruptedException Will throw exception if parameter is null
     */
//    private int explorationAllRightWallHug(Point start) throws InterruptedException {
//        boolean doingImage = false;
//        areaExplored = exploredMap.getExploredPercentage();
//        startTime = System.currentTimeMillis();
//
//        //Set stipulated end time
//        endTime = startTime + timeLimit;
//
//        double prevArea;
//        int moves = 1;
//        // Limit of moves that does not increment robot's exploration progress (i.e. no increased in percentage of
//        // explored map)
//        int checkingStep = RobotConstants.CHECKSTEPS;
//        this.start = start;
//
//        outer:
//        do {
//            prevArea = areaExplored;
//
//            if(areaExplored >= 100)
//                break;
//            try {
//                //Execute one movement of right wall hug
//                rightWallHug(doingImage);
//
//            } catch (InterruptedException e1) {
//                e1.printStackTrace();
//            }
//
//            areaExplored = exploredMap.getExploredPercentage();
//            //Check if areaExplored has increased
//            if (prevArea == areaExplored)
//                //Increment number of moves each iteration of this loop
//                moves++;
//                //If areaExplored has increased; progression
//            else
//                moves=1;
//
//            LOGGER.info(Double.toString(areaExplored));
//
//            //Prevent endless loop of moving right and forward in "cage-like" obstacle or no progression in exploration
//            if (moves % checkingStep == 0 || right_move > 3 || (robot.getPos().distance(start)==0 && areaExplored < 100.00)) {
//                do{
//                    //If robot position at start; do calibration
//                    if (robot.getPos().equals(start)) {
//                        goToPoint(start);
//                        calibrate_at_start_before_going_out();
//                    }
//                    prevArea = areaExplored;
//                    //If no path to go to nearest unexplored cell; stop carrying out right wall hug
//                    if(!goToUnexplored())
//                        break outer;
//                    areaExplored = exploredMap.getExploredPercentage();
//                }while(prevArea == areaExplored);
//                moves = 1;
//                checkingStep = RobotConstants.CHECKSTEPS;
//            }
//            //Have not reached coverage limit or time limit
//        } while (areaExplored < coverageLimit && System.currentTimeMillis() < endTime);
//
//        //Stop timer if coverage limit or time limit reached
//        if (sim) {
//            Main.SimulatorNew.displayTimer.stop();
//        }
//        moves = 0;
//        //Continue right wall hug for robot which have not reach no-progress limit in previous right wall hug
//        while (!robot.getPos().equals(start) && moves < checkingStep) {
//            rightWallHug(doingImage);
//            moves++;
//        }
//
//        robot.setImageCount(0);
//        //Continue second run to do image exploration; remove if not doing image reconigtion
//        robot.imageRecognitionRight(exploredMap);
//        //Calibrate the robot at start point
//        goToPoint(start);
//        //Compute run-time
//        endTime = System.currentTimeMillis();
//        int seconds = (int)((endTime - startTime)/1000%60);
//        int minutes = (int)((endTime - startTime)/1000/60);
//        int total_in_seconds = (int)((endTime - startTime)/1000);
//        System.out.println("Total Time: "+total_in_seconds+" seconds");
//        System.out.println("Total Time: "+minutes+"mins "+seconds+"seconds");
//        return total_in_seconds;
//    }

    /**
     * Exploration loop to move robot to new position using right wall hugging algorithm, and updates map in each
     * iteration. Exits exploration when coverage or time limit is reached, or when exploration is complete.
     *
     * @param start Coordinates of starting position
     * @return Run time of exploration
     * @throws InterruptedException Will throw exception if parameters is null
     */
    public int exploration(Point start) throws InterruptedException {
        int realEndTime;
        robot.setDoingImage(true);
        areaExplored = exploredMap.getExploredPercentage();
        startTime = System.currentTimeMillis();
        timeLimit = RobotConstants.IMG_TIME_LIMIT * 1000;
        endTime = startTime + timeLimit;
        boolean exploreMore = false;
        double prevArea;
        int moves = 1;
        int checkingStep = RobotConstants.CHECKSTEPS;
        this.start = start;

        outer:
        do {
            prevArea = areaExplored;
            if (areaExplored >= 100)
                break;
            try {
                System.out.println("Right wall hug");
                rightWallHug(false);
                if (robot.getPos().x == 1 && robot.getPos().y == 1) {
                    exploreMore = true;
                    areaExplored = exploredMap.getExploredPercentage();
                }

            } catch (InterruptedException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
            }
            areaExplored = exploredMap.getExploredPercentage();
            //No progression in exploration
            if (prevArea == areaExplored)
                moves++;
            else
                moves = 1;

            System.out.println("Area explored  = " + (areaExplored));

            //Prevent endless loop of moving right and forward in "cage-like" obstacle or no progression in
            //TODO: Change back if required
            if (exploreMore && areaExplored < coverageLimit) {
                LOGGER.info("ran explore more");
                robot.setStatus("Exploring more");
                while (areaExplored < coverageLimit) {
                    goToUnexplored2();
                    areaExplored = exploredMap.getExploredPercentage();
                }
            }
            if (moves % checkingStep == 0 || right_move > 3 || (robot.getPos().distance(start) == 0 && areaExplored < 100.00)) {
                do {
                    //Go back to start point
                    if (robot.getPos().equals(start)) {
                        goToPoint(start);
                        //Calibrate robot at start point before moving out again
//                        if (!sim) {
////                            robot.turnRightAndAlignMethodWithoutMapUpdate(exploredMap, realMap);
////                            robot.align_front(exploredMap, realMap);
////                            robot.align_right(exploredMap, realMap);
//                        }
                    }
                    prevArea = areaExplored;
                    //If cannot move to nearest unexplored cell, break
                    if (!goToUnexplored())
                        break outer;
                    areaExplored = exploredMap.getExploredPercentage();
                    //If no progression, attempt to recalibrate and move to unexplored area
                    //To stop recalibration from repeating, add counter to limit loop of calibration
                } while (prevArea == areaExplored);
                moves = 1;
                checkingStep = RobotConstants.CHECKSTEPS;
            }
            //Move to new position using right wall hug algorithm for each iteration
        } while (areaExplored < coverageLimit && System.currentTimeMillis() < (endTime+10*1000));
        if (sim) {
            Main.SimulatorNew.displayTimer.stop();
        }
        //Return to start point

       if(System.currentTimeMillis()<endTime) {
            goToPoint(start);
        }
        if (sim) {
            Main.SimulatorNew.displayTimer.stop();
        }
//        goToPoint(start);
        endTime = System.currentTimeMillis();
        int seconds = (int) ((endTime - startTime) / 1000 % 60);
        int minutes = (int) ((endTime - startTime) / 1000 / 60);
        int total_in_seconds = (int) ((endTime - startTime) / 1000);
        System.out.println("Total Time: " + total_in_seconds + " seconds");
        System.out.println("Total Time: " + minutes + "mins " + seconds + "seconds");
        for (int i = 0; i < robot.getImageResult().length(); i++) {
            System.out.println(robot.getImageResult().get(i).toString());
        }
        return total_in_seconds;
    }

    public int image_exploration(Point start) throws InterruptedException {

        areaExplored = exploredMap.getExploredPercentage();
        startTime = System.currentTimeMillis();
        endTime = startTime + timeLimit;

        boolean exploreMore = false;
        double prevArea;
        int moves = 1;
        int checkingStep = RobotConstants.CHECKSTEPS;
        this.start = start;
        robot.setDoingImage(true);
        outer:
        do {
            prevArea = areaExplored;
            if (areaExplored >= 100)
                break;
            try {
                System.out.println("Right wall hug");
                rightWallHug(false);
                if (robot.getPos().x == 1 && robot.getPos().y == 1) {
                    exploreMore = true;
                    areaExplored = exploredMap.getExploredPercentage();
                }

            } catch (InterruptedException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
            }
            areaExplored = exploredMap.getExploredPercentage();
            //No progression in exploration
            if (prevArea == areaExplored)
                moves++;
            else
                moves = 1;

            System.out.println("Area explored  = " + (areaExplored));

            //Prevent endless loop of moving right and forward in "cage-like" obstacle or no progression in
            if (exploreMore && areaExplored < coverageLimit) {
                LOGGER.info("ran explore more");
                robot.setStatus("Exploring more");
                while (areaExplored < coverageLimit) {
                    goToUnexplored2();
                    areaExplored = exploredMap.getExploredPercentage();
                }
            }
            if (moves % checkingStep == 0 || right_move > 3 || (robot.getPos().distance(start) == 0 && areaExplored < 100.00)) {
                do {
                    //Go back to start point
                    if (robot.getPos().equals(start)) {
                        goToPoint(start);
                    }
                    prevArea = areaExplored;
                    //If cannot move to nearest unexplored cell, break
                    if (!goToUnexplored())
                        break outer;
                    areaExplored = exploredMap.getExploredPercentage();
                    //If no progression, attempt to recalibrate and move to unexplored area
                    //To stop recalibration from repeating, add counter to limit loop of calibration
                } while (prevArea == areaExplored);
                moves = 1;
                checkingStep = RobotConstants.CHECKSTEPS;
            }
            //Move to new position using right wall hug algorithm for each iteration
        } while (areaExplored < coverageLimit && System.currentTimeMillis() < endTime);

//        //Return to start point
//        goToPoint(start);
//
//        System.out.println("Back at start point");
//
//        //Image rec part
//        //Initialize all obstacle surfaces and add in obsSurfaces
//        System.out.println("Creating obs surfaces");
//        createObstacleSurfaces(exploredMap);
//        System.out.println("No of obstacle surfaces = " + obsSurfaces.size());
//        System.out.println("Finished creating obs surfaces");
//        Go through obsSurfaces array, go to point for each of them
//        System.out.println(robot.getObsSurfaces().size());
//        obsSurfaces = robot.getObsSurfaces();
//        robot.setImageRec(true);
        System.out.println("Starting to go to obstacle surfaces");
        goToObstacleSurfaces(exploredMap);
        //Go back to start point
        goToPointWithoutSensing(start);
        for (int i = 0; i < robot.getImageResult().length(); i++) {
            System.out.println(robot.getImageResult().get(i).toString());
        }

        if (sim) {
            Main.SimulatorNew.displayTimer.stop();
        }
        endTime = System.currentTimeMillis();
        int seconds = (int) ((endTime - startTime) / 1000 % 60);
        int minutes = (int) ((endTime - startTime) / 1000 / 60);
        int total_in_seconds = (int) ((endTime - startTime) / 1000);
        System.out.println("Total Time: " + total_in_seconds + " seconds");
        System.out.println("Total Time: " + minutes + "mins " + seconds + "seconds");
        return total_in_seconds;

    }


    public void createObstacleSurfaces(Map exploredMap) {
        int rowInc, colInc, tempRow, tempCol;
        Cell tempCell, tempCell2, tempCell3;
        for (int i = 0; i < MapConstants.MAP_HEIGHT; i++) {
            for (int j = 0; j < MapConstants.MAP_WIDTH; j++) {
                tempCell = exploredMap.getCell(i, j);
                System.out.println("For cell " + i + "," + j);
                //Add obstacle surfaces for all obstacle in explored map
                if (tempCell.isObstacle()) {
                    System.out.println("Cell is obstacle");
                    //For each direction, get the increment for row/col
                    Direction dir = Direction.UP;
                    //Consider all possible surfaces of obstacle
                    for (int k = 0; k < 4; k++) {
                        dir = Direction.getClockwise(dir);
                        System.out.println("For direction no: " + k);
                        rowInc = getRowIncrementForMovement(dir);
                        colInc = getColIncrementForMovement(dir);
                        //Check if each obstacle surface is possible to detect (need to have two free spaces in front for image rec)
                        for (int l = 1; l <= RobotConstants.CAMERA_RANGE; l++) {
                            tempRow = tempCell.getPos().y + rowInc * l;
                            tempCol = tempCell.getPos().x + colInc * l;
                            if (exploredMap.checkValidCell(tempRow, tempCol)) {
//                                System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is valid");
                                tempCell2 = exploredMap.getCell(tempRow, tempCol);
                                if (tempCell2.isObstacle()) {
//                                    System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is obstacle");
                                    break;
                                } else {
//                                    System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is NOT obstacle");
                                    //Both cells directly in front of obstacle surface is empty, and the target cell is
                                    // not virtual wall; add to obstacle surface
                                    if (l == RobotConstants.CAMERA_RANGE) {
                                        if (!tempCell2.isVirtualWall()) {
                                            System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is NOT obstacle OR virtual wall");

                                            ObsSurface obsSurface = new ObsSurface(tempCell.getPos(), tempCell2.getPos(), dir, Direction.getOpposite(dir));
                                            obsSurfaces.add(obsSurface);
                                            System.out.println("Created obstacle surface");
                                        } else {
                                            ArrayList<Cell> possibleNeighbours = exploredMap.getNeighbours(tempCell, dir);
                                            System.out.print("Neighbours of Cell: " + tempCell.getPos().x + " , " + tempCell.getPos().y + "\n");
                                            System.out.print(possibleNeighbours.size());
                                            for (int m = 0; m < possibleNeighbours.size(); m++) {

                                                Cell neighbourCell = possibleNeighbours.get(m);
                                                for (int n = 1; n <= RobotConstants.CAMERA_RANGE; n++) {
                                                    tempRow = neighbourCell.getPos().y + rowInc * n;
                                                    tempCol = neighbourCell.getPos().x + colInc * n;
                                                    if (exploredMap.checkValidCell(tempRow, tempCol)) {
                                                        tempCell2 = exploredMap.getCell(tempRow, tempCol);
                                                        if (tempCell2.isObstacle()) {
                                                            break;
                                                        } else {
                                                            if (n == RobotConstants.CAMERA_RANGE) {
                                                                if (!tempCell2.isVirtualWall()) {
                                                                    System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is NOT obstacle OR virtual wall");

                                                                    ObsSurface obsSurface = new ObsSurface(tempCell.getPos(), tempCell2.getPos(), dir, Direction.getOpposite(dir));
                                                                    obsSurfaces.add(obsSurface);
                                                                    System.out.println("Created obstacle surface");
                                                                }
                                                            }
                                                        }
                                                    } else {
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {
                                System.out.println("Cell no " + l + " in front of obstacle in direction " + k + " is INVALID");
                                break;
                            }
                        }
                    }
                }
                System.out.println("is not obstacle");
            }
        }
    }

    public ArrayList<ObsSurface> removeNeighbouringObsSurfaces(Map exploredMap, ObsSurface targetObsSurface) {
        ArrayList<ObsSurface> neighbouringObsSurfaces = new ArrayList<ObsSurface>();
        Point tempPos;
        Direction tempDir;
        ObsSurface tempObsSurface;
        for (int i = 0; i < obsSurfaces.size(); i++) {
            tempObsSurface = obsSurfaces.get(i);
            tempPos = tempObsSurface.getPos();
            if ((Math.abs(tempPos.x - targetObsSurface.getPos().x) == 1 && Math.abs(tempPos.y - targetObsSurface.getPos().y) == 0) || (Math.abs(tempPos.x - targetObsSurface.getPos().x) == 0 && Math.abs(tempPos.y - targetObsSurface.getPos().y) == 1)) {
                tempDir = tempObsSurface.getSurface();
                if (targetObsSurface.getSurface().equals(tempDir)) {
                    obsSurfaces.remove(obsSurfaces.get(i));
                }
            }

        }
        return neighbouringObsSurfaces;
    }

    public boolean isNeighbouringCell(Point point1, Point point2) {
        int x1, x2, y1, y2, xDiff, yDiff;
        x1 = point1.x;
        y1 = point1.y;
        x2 = point2.x;
        y2 = point2.y;
        return true;

    }

    public boolean goToObstacleSurfaces(Map exploredMap) throws InterruptedException {
        ObsSurface targetObsSurface;
        String imgFileName, imgResult;
        //Ensure that robot goes to all obstacle surfaces
//        ArrayList<ObsSurface> obsSurfaces = robot.getObsSurfaces();
        while (obsSurfaces.size() > 0) {
            System.out.print("No of surfaces left" + obsSurfaces.size());
            targetObsSurface = exploredMap.nearestObsSurface(robot.getPos(), obsSurfaces);
            System.out.print("Obstacle location" + targetObsSurface.getPos().x + "," + targetObsSurface.getPos().y);

            //Execute movements to obstacle surface point to take image
            if (!goToPointWithoutSensing(targetObsSurface.getTargetPos())) {
                return false;
            }
            while (robot.getDir() != targetObsSurface.getTargetDir()) {
                robot.turn(Command.TURN_RIGHT, 1);
                robot.senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
            }
            //Robot facing obstacle surface
            //Take image -> Image Rec -> Process result -> Add to image string if surface detected
            if (sim) {
                robot.setStatus("Send image command to Rpi");
                System.out.println("Send image command to Rpi");
                TimeUnit.MILLISECONDS.sleep(750);
            } else {

            }
            removeNeighbouringObsSurfaces(exploredMap, targetObsSurface);
            obsSurfaces.remove(targetObsSurface);


        }
        return true;
    }


    private boolean goToUnexplored2() throws InterruptedException {
        robot.setStatus("Go to nearest unexplored\n");
        LOGGER.info(robot.getStatus());


        Cell nearestUnexplored = exploredMap.nearestUnexplored(robot.getPos());
        LOGGER.info("Nearest unexplored: " + nearestUnexplored);


        Cell nearestExp = exploredMap.nearestExplored(nearestUnexplored.getPos(), robot.getPos());
        LOGGER.info("Nearest explored: " + nearestExp);
        if (nearestExp == null) {
            LOGGER.info("No nearest unexplored found.");
            return false;
        } else {
            robot.setStatus("Go to nearest explored " + nearestExp.getPos().toString() + "\n");
            LOGGER.info("Go to " + nearestExp.toString());
            return goToPoint2(nearestExp.getPos());
        }
    }

    /**
     * Robot move to nearest unexplored cell
     *
     * @return True if there is an unexplored cell robot moves to unexplored cell, false if no such cell or no path to
     * reach this cell
     */
    private boolean goToUnexplored() throws InterruptedException {
        robot.setStatus("Go to nearest unexplored\n");
        LOGGER.info(robot.getStatus());


        Cell nearestUnexplored = exploredMap.nearestUnexplored(robot.getPos());
        LOGGER.info("Nearest unexplored: " + nearestUnexplored);

        Cell nearestExp = exploredMap.nearestExplored(nearestUnexplored.getPos(), robot.getPos());
        LOGGER.info("Nearest explored: " + nearestExp);
        if (nearestExp == null) {
            LOGGER.info("No nearest unexplored found.");
            return false;
        } else {
            robot.setStatus("Go to nearest explored " + nearestExp.getPos().toString() + "\n");
            LOGGER.info("Go to " + nearestExp.toString());
            return goToPoint(nearestExp.getPos());
        }
    }


    /**
     * Execute different sensing operations for image recognition and exploration
     *
     * @param doingImage True for image recognition, false otherwise
     */
    private void senseForExplorationOrImage(boolean doingImage) {
        ArrayList<ObsSurface> surfTaken;
//        if (doingImage) {
//            surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//            updateNotYetTaken(surfTaken);
//        }
//        else {
        robot.sense(exploredMap, realMap);
//        }
    }

    public void executeImageRecFront() throws InterruptedException {

        if(!imagePositionPossible(exploredMap, Direction.getOpposite(robot.getDir()),robot.getPos())) {
            System.out.println("Image rec front not possible due to position");
            return;
        }
        if(robot.checkIfCapturedPosition(robot.getPos(),robot.getDir())){
            return;
        }

        System.out.println("Image Rec Front");
        String imgFileName, imgResult;

        Point refPoint = new Point((robot.getPos().x + getColIncrementForMovement(robot.getDir()) * 2), (robot.getPos().y + getRowIncrementForMovement(robot.getDir()) * 2));
        System.out.println("Reference point :" + refPoint.x + "," + refPoint.y);
        Direction refDir = Direction.getOpposite(robot.getDir());
        ObsSurface targetObsSurface = new ObsSurface(refPoint, refDir);

        if (!sim) {
            imgFileName = robot.takeImg();
            imgResult = robot.rpiImageRec(imgFileName);
            System.out.println("Image String received is" + imgResult);
            processImgResult(targetObsSurface, imgResult);
            robot.addCapturedPosition(robot.getPos(), robot.getDir());
        } else {
            robot.setStatus("Send image command to Rpi");
            System.out.println("Send image command to Rpi");
            TimeUnit.MILLISECONDS.sleep(750);
        }
        System.out.println("Image Rec Front DONE");

    }

    public void executeImageRecRight() throws InterruptedException {
        System.out.println("Image rec front not possible due to position");
        if(!imagePositionPossible(exploredMap, Direction.getAntiClockwise(robot.getDir()),robot.getPos())) {
            return;
        }

        if(robot.checkIfCapturedPosition(robot.getPos(),Direction.getAntiClockwise(robot.getDir()))){
            return;
        }


        System.out.println("Image Rec Right");
        String imgFileName, imgResult;

        Point refObsPoint = new Point((robot.getPos().x + getColIncrementForMovement(Direction.getClockwise(robot.getDir())) * 2), (robot.getPos().y + getRowIncrementForMovement(Direction.getClockwise(robot.getDir())) * 2));
        System.out.println("Reference point :" + refObsPoint.x + "," + refObsPoint.y);
        Direction refDir = Direction.getAntiClockwise(robot.getDir());
        ObsSurface targetObsSurface = new ObsSurface(refObsPoint, refDir);
        robot.turn(Command.TURN_RIGHT, 1);
        robot.sense(exploredMap, realMap);
        robot.align_front(exploredMap, realMap);
        if (!sim) {
            imgFileName = robot.takeImg();
            imgResult = robot.rpiImageRec(imgFileName);
            processImgResult(targetObsSurface, imgResult);
            robot.addCapturedPosition(robot.getPos(), Direction.getAntiClockwise(robot.getDir()));
        } else {
            robot.setStatus("Send image command to Rpi");
            System.out.println("Send image command to Rpi");
            TimeUnit.MILLISECONDS.sleep(750);
        }

        robot.turn(Command.TURN_LEFT, 1);
        robot.sense(exploredMap, realMap);
        robot.align_front(exploredMap, realMap);
        robot.obstacleStepsCounter = 0;
    }

    /**
     * Right wall hug algorithm - order of preference of movement (highest preference to lowest preference)
     * 1. Turn right and move one cell (if possible)
     * 2. Move forward and move one cell
     * 3. Turn left and move one cell
     * 4. U-turn
     *
     * @param doingImage True for image recognition, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private void rightWallHug(boolean doingImage) throws InterruptedException {
        boolean aligned_front;

//        if (robot.getPos().x == 13 && robot.getPos().y == 18) {
//            System.out.println("Aligning at goal zone");
//            System.out.println(robot.getDir());
//            String calibrationCmd = robot.getCommand(Command.INITIAL_CALIBRATE, 1);
//            switch (robot.getDir()) {
//                case UP:
//                    robot.turn(Command.TURN_RIGHT, stepPerSecond);
//                    robot.sense(exploredMap, realMap);
//                    System.out.println("Turning Right and doing inital calibrate");
//                    NetMgr.getInstance().send(NetworkConstants.ARDUINO + calibrationCmd);
//                case RIGHT:
//                    System.out.println("Doing initial calibrate");
//                    NetMgr.getInstance().send(NetworkConstants.ARDUINO + calibrationCmd);
//            }
//        }

        Direction robotDir = robot.getDir();
        //Check if right movement is possible
        if (movable(Direction.getClockwise(robotDir))) {

            //Execute image rec front if facing obstacle
            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                System.out.println("Testing for image position possible");
                if(imagePositionPossible(exploredMap, Direction.getOpposite(robot.getDir()),robot.getPos()))
                executeImageRecFront();
            }
            //On obstacle and steps count is more than zero-> outstanding
            if (!robot.isRightHuggingWall()) {
                robot.obstacleSide++;
                robot.obstacleStepsCounter = 0;

            }

            robot.turn(Command.TURN_RIGHT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            //TODO: Revert back when doing image
            robot.sense(exploredMap, realMap);
            moveForward(RobotConstants.MOVE_STEPS, stepPerSecond, doingImage);
            right_move++;
            robot.obstacleStepsCounter++;
        }
        //Check if forward movement is possible
        else if (movable(robotDir)) {

            robot.move(Command.FORWARD, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
            robot.sense(exploredMap, realMap);
            if (!robot.isRightHuggingWall()) {
                robot.obstacleStepsCounter++;
            }
            if (robot.obstacleStepsCounter == 3 && robot.isDoingImage() && robot.checkFrontForSingleObstacle(exploredMap, Direction.getClockwise(robot.getDir()))) {
                robot.align_front(exploredMap, realMap);
                executeImageRecRight();
            }
            right_move = 0;
        }

        //Check if can move in left direction
        else if (movable(Direction.getAntiClockwise(robotDir))) {


            if (!sim) {
                if (robot.getAlignCount() == 0) {
                    robot.align_right(exploredMap, realMap);
                }
            }
            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                executeImageRecFront();
                robot.obstacleSide = 1;
            }
            if (!robot.isRightHuggingWall()) {
                robot.obstacleSide = 1;
                if (robot.obstacleStepsCounter > 0 && robot.isDoingImage()) {
                    robot.align_front(exploredMap, realMap);
                    executeImageRecRight();
                }
                robot.obstacleStepsCounter = 0;
            } else {
                robot.obstacleStepsCounter = 1;
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            //TODO: Revert back when doing image
            robot.sense(exploredMap, realMap);

            moveForward(RobotConstants.MOVE_STEPS, stepPerSecond, doingImage);
            right_move = 0;

        }

        //If all fails, u-turn
        else {

            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                executeImageRecFront();
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            //TODO: Revert back when doing image
            robot.sense(exploredMap, realMap);

            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                executeImageRecFront();
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            //TODO: Revert back when doing image
            robot.sense(exploredMap, realMap);

        }
    }

    private boolean backRightCellisObstacleOrWall(Map exploredMap) {
        int rowDiff = 0, colDiff = 0;
        switch (robot.getDir()) {
            case UP: {
                //colDiff is x
                //rowDiff is y
                rowDiff = -1;
                colDiff = 2;
                break;
            }
            case RIGHT: {
                rowDiff = -2;
                colDiff = -1;
                break;
            }
            case LEFT: {
                rowDiff = 2;
                colDiff = 1;
                break;
            }
            case DOWN: {
                rowDiff = 1;
                colDiff = -2;
                break;
            }
        }
        if (exploredMap.checkValidCell((robot.getPos().x + colDiff), (robot.getPos().y + rowDiff))) {
            return exploredMap.getCell((robot.getPos().x + colDiff), (robot.getPos().y + rowDiff)).isObstacle();
        }
        return robot.isRightHuggingWall();
    }
//    private boolean checkFrontForObstacleOrRightWall(Map exploredMap){
//        Point F1= new Point();
//        Point F2= new Point();
//        Point F3= new Point();
//        switch(robot.getDir()){
//            case UP: {
//                F1.x = robot.getPos().x -1;
//                F1.y = robot.getPos().y +2;
//                F2.x = robot.getPos().x;
//                F2.y = robot.getPos().y+2;
//                F3.x = robot.getPos().x +1;
//                F3.y = robot.getPos().y+2;
//                break;
//            }
//            case RIGHT:{
//                F1.x = robot.getPos().x +2;
//                F1.y = robot.getPos().y +1;
//                F2.x = robot.getPos().x +2;
//                F2.y = robot.getPos().y;
//                F3.x = robot.getPos().x +2;
//                F3.y = robot.getPos().y-1;
//                break;
//            }
//            case DOWN:{
//                F1.x = robot.getPos().x -1;
//                F1.y = robot.getPos().y -2;
//                F2.x = robot.getPos().x;
//                F2.y = robot.getPos().y -2;
//                F3.x = robot.getPos().x +1;
//                F3.y = robot.getPos().y -2;
//                break;
//            }
//            case LEFT:{
//                F1.x = robot.getPos().x -2;
//                F1.y = robot.getPos().y +1;
//                F2.x = robot.getPos().x -2;
//                F2.y = robot.getPos().y;
//                F3.x = robot.getPos().x -2;
//                F3.y = robot.getPos().y+1;
//                break;
//            }
//        }
//
//        int F2row = robot.getSensor("F2").getRow();
//        int F2col = robot.getSensor("F2").getCol();
//        if(F2row == 0 || F2row == MapConstants.MAP_HEIGHT-1 || F2col == 0 || F2col == MapConstants.MAP_WIDTH-1) {
//            return true;
//        }
//        else if(exploredMap.checkValidCell(F1.x, F1.y) && exploredMap.checkValidCell(F3.x,F3.y)){
//            return exploredMap.getCell(F1.x, F1.y).isObstacle() && exploredMap.getCell(F3.x, F3.y).isObstacle();
//        }
//        return false;
//    }

    /**
     * Turn right and calibrate front sensors to align with obstacle/wall; turn left and calibrate right sensors to
     * align with obstacle/wall
     * Avoid turning twice with turnAndAlignCount in Robot class
     *
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private void turnRightAndAlignBeforeTurnLeft(boolean doingImage) throws InterruptedException {

        //If right obstacle/wall on right side of robot, real exploration and has not turned and aligned yet
        if ((robot.getSensorRes().get("R1") == 1) &&
                (!robot.getHasTurnAndAlign()) &&
                (!sim)) {
            LOGGER.info("turnRightAndAlignBeforeTurnLeft");
            //If doing image recognition, calibrate and capture obstacle surface without updating map
//            if (doingImage) {
//                robot.turnRightAndAlignMethodWithoutMapUpdate(exploredMap, realMap);
//            }
//            //If doing exploration only, just calibrate
//            else {
            robot.turnRightAndAlignMethod(exploredMap, realMap);
        }
    }
    //Has already turned and aligned
//        else if (robot.getHasTurnAndAlign()) {
//            robot.setHasTurnAndAlign(false);
//        }
//    }

    /**
     * Calibrate and capture obstacle surfaces of right obstacle/wall before turning
     * @param doingImage True if doing image recognition, false otherwise
     */
//    private void alignAndImageRecBeforeLeftTurn(boolean doingImage) {
//        if (!sim) {
//            //Calibrate robot
//            robot.align_front(exploredMap, realMap);
//            robot.align_right(exploredMap, realMap);
//            //Capture obstacle surface before turning left
//            robot.setImageCount(0);
//            //TODO: Uncomment when doing image recognition
//    //            ArrayList<ObsSurface> surfTaken = robot.imageRecognitionRight(exploredMap);
////            if (doingImage) {
////                updateNotYetTaken(surfTaken);
////            }
//        }
//    }

    /**
     * Move forward in number of steps, sensing robot's environment after each step (if movable)
     *
     * @param steps Number of steps to move forward
     */
    private void moveForward(int steps, int stepPerSecond, boolean doingImage) throws InterruptedException {
        if (movable(robot.getDir())) {       // for actual, double check in case of previous sensing error


            robot.move(Command.FORWARD, steps, exploredMap, stepPerSecond);
//            if (doingImage) {
//                ArrayList<ObsSurface> surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//            }
//            else {
            robot.sense(exploredMap, realMap);
//            }
        }
    }

    /**
     * Check if the next move in cells of that direction is a valid move
     *
     * @param dir Direction of intended movement w.r.t robot
     * @return True if movable, false otherwise
     */

    private boolean movable(Direction dir) {
        LOGGER.info("movable");
        int rowInc = getRowIncrementForMovement(dir);
        int colInc = getColIncrementForMovement(dir);

        return exploredMap.checkValidMove(robot.getPos().y + rowInc, robot.getPos().x + colInc);
    }

    /**
     * Check if robot and target location is at start position; rotates robot to face down if true
     *
     * @param loc Coordinates of target location
     * @return True if robot and target location is both at start location, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private boolean robotAndTargetAtStartPos(Point loc) throws InterruptedException {
        if (robot.getPos().equals(start) && loc.equals(start)) {
            while (robot.getDir() != Direction.DOWN) {
                robot.turn(Command.TURN_LEFT, stepPerSecond);
                System.out.println(robot.getDir());
                if (sim) {
                    robot.sense(exploredMap, realMap);
                } else {
                    NetMgr.getInstance().receive();
                }
            }
            return true;
        }
        return false;
    }

    /**
     * Execute commands generated by a star algorithm for fastest path to target location; senses and update map after
     * every move
     *
     * @param commands Array of commands to be executed in order
     * @param loc      Coordinates of target location
     * @throws InterruptedException Will throw exception if parameter(s) is null
     */
    private void executeCommandsMoveToTargetWithoutSensing(ArrayList<Command> commands, Point loc) throws InterruptedException {
        for (Command c : commands) {
            System.out.println("Command: " + c);
            if ((c == Command.FORWARD) && !movable(robot.getDir())) {
                System.out.println("Not Executing Forward Not Movable");
                // Recompute a star path to location
                goToPoint(loc);
                break;

            } else {
                //If last command is turn; robot has already reached the destination point
                if (((c == Command.TURN_LEFT && !movable(Direction.getAntiClockwise(robot.getDir()))) ||
                        (c == Command.TURN_RIGHT && !movable(Direction.getClockwise(robot.getDir())))) && commands.indexOf(c) == commands.size() - 1)
                    continue;
                //Calibrate before turn
                if (c == Command.TURN_LEFT || c == Command.TURN_RIGHT) {
//                    alignAndImageRecBeforeLeftTurn(false);
//                    if(!sim){
                    robot.align_front(exploredMap, realMap);
//                    }
                    robot.turn(c, stepPerSecond);
                }
                //Continue otherwise
                else {
                    robot.move(c, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
                }

                robot.senseWithoutMapUpdate(exploredMap, realMap);
            }
        }
    }

    /**
     * Execute commands generated by a star algorithm for fastest path to target location; senses and update map after
     * every move
     *
     * @param commands Array of commands to be executed in order
     * @param loc      Coordinates of target location
     * @throws InterruptedException Will throw exception if parameter(s) is null
     */
    private void executeCommandsMoveToTarget(ArrayList<Command> commands, Point loc) throws InterruptedException {
        for (Command c : commands) {
            System.out.println("Command: " + c);
            if ((c == Command.FORWARD) && !movable(robot.getDir())) {
                System.out.println("Not Executing Forward Not Movable");
                // Recompute a star path to location
                goToPoint(loc);
                break;

            } else {
                //If last command is turn; robot has already reached the destination point
                if (((c == Command.TURN_LEFT && !movable(Direction.getAntiClockwise(robot.getDir()))) ||
                        (c == Command.TURN_RIGHT && !movable(Direction.getClockwise(robot.getDir())))) && commands.indexOf(c) == commands.size() - 1)
                    continue;
                //Calibrate before turn
                if (c == Command.TURN_LEFT || c == Command.TURN_RIGHT) {
//                    alignAndImageRecBeforeLeftTurn(false);
//                    if(!sim){
                    robot.align_front(exploredMap, realMap);
//                    }
                    robot.turn(c, stepPerSecond);
                }
                //Continue otherwise
                else {
                    robot.move(c, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
                }

                robot.sense(exploredMap, realMap);
            }
        }
    }
    /**
     * Execute commands generated by a star algorithm for fastest path to target location; senses and does not update map after
     * every move. Recompute path to nearest unexplored point if command is forward but invalid.
     * @param commands Array of commands to be executed in order
     * @param loc Coordinates of target location
     * @throws InterruptedException Will throw exception if parameter(s) is null
     */

//    private void executeCommandsMoveToTarget(ArrayList<Command> commands, Point loc, ObsSurface obsSurface) throws InterruptedException{
//        ArrayList<ObsSurface> surfTaken;
//        for (Command c : commands) {
//            System.out.println("Command: " + c);
//            if ((c == Command.FORWARD) && !movable(robot.getDir())) {
//                System.out.println("Not Executing Forward Not Movable");
//                // Recompute a star path to location
//                goToPointForImage(loc, obsSurface);
//                break;
//            } else {
//                if (((c == Command.TURN_LEFT && !movable(Direction.getAntiClockwise(robot.getDir()))) ||
//                        (c == Command.TURN_RIGHT && !movable(Direction.getClockwise(robot.getDir())))) && commands.indexOf(c) == commands.size() - 1)
//                    goToPointForImage(loc, obsSurface);
//                if (c == Command.TURN_LEFT || c == Command.TURN_RIGHT) {
//                    robot.turn(c, stepPerSecond);
//                } else {
//                    robot.move(c, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
//                }
//
//                surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//            }
//        }
//    }

    /**
     * Make sure robot face intended direction such that obstacle surface is on the right of the robot. Captures
     * obstacle and updates surfaces that is not taken
     * @param desiredDir Intended direction
     * @throws InterruptedException Will throw exception if parameter is null
     */

//    private void turnRobotToFaceDirectionDuringImage(Direction desiredDir) throws InterruptedException{
//        ArrayList<ObsSurface> surfTaken;
//        //If robot already facing desired direction, return
//        if (desiredDir != robot.getDir()) {
//            //Desired direction on robot's right turn, turn and capture surfaces and update surface
//            if (desiredDir == Direction.getClockwise(robot.getDir())) {
//                robot.turn(Command.TURN_RIGHT, stepPerSecond);
//                surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//            }
//            //Desired direction on robot's left turn, turn and capture surfaces and update surface
//            else if (desiredDir == Direction.getAntiClockwise(robot.getDir())) {
//                robot.turn(Command.TURN_LEFT, stepPerSecond);
//                surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//            }
//            //Desired direction on robot's u turn, turn twice and capture surfaces and update surfaces
//            else {
//                robot.turn(Command.TURN_LEFT, stepPerSecond);
//                surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//                robot.turn(Command.TURN_LEFT, stepPerSecond);
//                surfTaken = robot.senseWithoutMapUpdate(exploredMap, realMap);
//                updateNotYetTaken(surfTaken);
//            }
//        }
//    }

    /**
     * Moves robot to nearest virtual wall after reaching unexplored cell; continue right wall hugging algorithm upon
     * reaching virtual wall
     *
     * @throws InterruptedException Will throw exception if parameters is null
     */
    private void continueExplorationUponNearestUnexplored() throws InterruptedException {
        robot.setStatus("Continue exploration, finding the nearest virtual wall.");
        LOGGER.info(robot.getStatus());

        //Get direction of the nearest virtual wall
        Direction dir = nearestVirtualWall(robot.getPos());
        System.out.println(dir);

        //If can move in the direction of nearest virtual wall, turn robot to face direction
        if (movable(dir)) {
            while (dir != robot.getDir()) {
                if (dir.ordinal() - robot.getDir().ordinal() == 1)
                    robot.turn(Command.TURN_LEFT, stepPerSecond);
                else
                    robot.turn(Command.TURN_RIGHT, stepPerSecond);
            }

            //Keep moving in the same direction until unable (meet obstacle at the front)
            while (movable(robot.getDir())) {
                robot.move(Command.FORWARD, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
                robot.sense(exploredMap, realMap);
            }
        }

        //Re-orientate robot's direction until it is right-hugging obstacle
        while (Direction.getAntiClockwise(dir) != robot.getDir()) {
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.sense(exploredMap, realMap);
        }

    }

    /**
     * Execute commands generated by a star algorithm for fastest path to target location, senses and update map after
     * every move; if unable to execute command (obstacle in direction of command), call goToPoint algorithm
     * to generate path from robot's current location to target location
     *
     * @param commands Array of commands to be executed in order
     * @param loc      Coordinates of start point
     * @throws InterruptedException Will throw exception if parameter(s) is null
     */
    private void executeCommandsMoveToStartPoint(ArrayList<Command> commands, Point loc) throws InterruptedException {
        //Moves to calculate no of steps to take forward
        int moves = 0;
        Command c;
        for (int i = 0; i < commands.size(); i++) {
            c = commands.get(i);

            // Command is forward but unable to move forward; update map and recalculate path from current location
            // to target location using goToPointFuncation
            if ((c == Command.FORWARD) && (robot.getSensorRes().get("F1") == 1 ||
                    robot.getSensorRes().get("F2") == 1 || robot.getSensorRes().get("F3") == 1)
            ) {
                System.out.println("Not Executing Forward Not Movable");
                //Update map after sensing
                robot.updateMap(exploredMap, robot.getSensorRes());
                goToPoint(loc);
                break;
            } else {
                //Increment number of steps to move forward
                if (c == Command.FORWARD && moves < 1) {
                    moves++;
                    // For last command, no need map update and alignment
                    if (i == (commands.size() - 1)) {
                        robot.move(c, moves, exploredMap, stepPerSecond);
                        robot.senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
                    }
                } else {
                    // Move foward by no of steps in counter
                    if (moves > 0) {
                        robot.move(Command.FORWARD, moves, exploredMap, stepPerSecond);
                        robot.senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
                    }
                    if (c == Command.TURN_RIGHT || c == Command.TURN_LEFT) {
                        robot.turn(c, stepPerSecond);
                    }
                    //Execute command
                    else {
                        robot.move(c, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
                    }
                    //Sense and update map
                    robot.senseWithoutMapUpdateAndAlignment(exploredMap, realMap);
                    //Reset move to 0
                    moves = 0;
                }
            }
        }
    }


    /**
     * Moves the robot to a specific point in the arena
     *
     * @param loc Coordinates of the point intended for robot to move to
     * @return True if movement is successful, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private boolean goToPoint(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetAtStartPos(loc)) {

            ArrayList<Command> commands;
            ArrayList<Cell> path;
            FastestPath fp = new FastestPath(exploredMap, robot, sim);
            //Run aStar algorithm for robot to reach target location
            path = fp.runAStar(robot.getPos(), loc, robot.getDir());
            //Return false if no viable path from robot's current position to target location
            if (path == null)
                return false;
            fp.displayFastestPath(path, true);
            commands = fp.getPathCommands(path);
            System.out.println("Exploration Fastest Commands: " + commands);

            //Not moving back to start single moves
            if (!loc.equals(start)) {
                executeCommandsMoveToTarget(commands, loc);
                //Sense environment after movement
                //If robot moved to nearest unexplored area and still not finished exploration; find nearest virtual wall and continue exploration
                if (exploredMap.getExploredPercentage() < 100 && movable(Direction.getClockwise(robot.getDir()))) {
                    continueExplorationUponNearestUnexplored();
                }
            }

            //Return to start position
            else {
                executeCommandsMoveToStartPoint(commands, loc);
            }
            //TODO: Might have problems when returning true/false from recursion call
        }
        //Robot successfully reached target location; return true
        return true;
    }

    /**
     * Moves the robot to a specific point in the arena
     *
     * @param loc Coordinates of the point intended for robot to move to
     * @return True if movement is successful, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private boolean goToPoint2(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetAtStartPos(loc)) {

            ArrayList<Command> commands;
            ArrayList<Cell> path;
            FastestPath fp = new FastestPath(exploredMap, robot, sim);
            //Run aStar algorithm for robot to reach target location
            path = fp.runAStar(robot.getPos(), loc, robot.getDir());
            //Return false if no viable path from robot's current position to target location
            if (path == null)
                return false;
            fp.displayFastestPath(path, true);
            commands = fp.getPathCommands(path);
            System.out.println("Exploration Fastest Commands: " + commands);

            //Not moving back to start single moves
            if (!loc.equals(start)) {
                executeCommandsMoveToTarget(commands, loc);
                //Sense environment after movement
                //If robot moved to nearest unexplored area and still not finished exploration; find nearest virtual wall and continue exploration
            }

            //Return to start position
            else {
                executeCommandsMoveToStartPoint(commands, loc);
            }
            //TODO: Might have problems when returning true/false from recursion call
        }
        //Robot successfully reached target location; return true
        return true;
    }

    /**
     * Moves the robot to a specific point in the arena without sensing
     *
     * @param loc Coordinates of the point intended for robot to move to
     * @return True if movement is successful, false otherwise
     * @throws InterruptedException Will throw exception if parameter is null
     */
    private boolean goToPointWithoutSensing(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetAtStartPos(loc)) {

            ArrayList<Command> commands;
            ArrayList<Cell> path;
            FastestPath fp = new FastestPath(exploredMap, robot, sim);
            //Run aStar algorithm for robot to reach target location
            path = fp.runAStar(robot.getPos(), loc, robot.getDir());
            //Return false if no viable path from robot's current position to target location
            if (path == null)
                return false;
            fp.displayFastestPath(path, true);
            commands = fp.getPathCommands(path);
            System.out.println("Exploration Fastest Commands: " + commands);

            //Not moving back to start single moves
//            if (!loc.equals(start)) {
            executeCommandsMoveToTargetWithoutSensing(commands, loc);
            //Sense environment after movement
            //If robot moved to nearest unexplored area and still not finished exploration; find nearest virtual wall and continue exploration
//            }

            //Return to start position
//            else {
//                executeCommandsMoveToStartPoint(commands, loc);
//            }
//            TODO: Might have problems when returning true/false from recursion call
        }
        //Robot successfully reached target location; return true
        return true;
    }

    /**
     * Checks cell in up,down,left,right direction w.r.t robot in straight line; returns direction to nearest virtual
     * wall
     *
     * @param pos Coordinates of robot's postion
     * @return Direction to nearest virtual wall
     */
    //Returns the direction to the nearest virtual wall
    private Direction nearestVirtualWall(Point pos) {
        int rowInc, colInc, lowest = 1000, lowestIter = 0, curDist;
        //Priority of direction of wall; right, up, left, down
        Direction dir = Direction.RIGHT;
        //Evaluate the distance to nearest virtualwall
        System.out.println("Nearest Wall");
        for (int i = 0; i < 4; i++) {
            //Row and column increment for up, right, left, right direction
            rowInc = (int) Math.sin(Math.PI / 2 * i);
            colInc = (int) Math.cos(Math.PI / 2 * i);
            curDist = 0;
            //Circularly check for nearest virtual wall, incrementing distance for each iteration
            for (int j = 1; j < MapConstants.MAP_HEIGHT; j++) {
                if (exploredMap.checkValidCell(pos.y + rowInc * j, pos.x + colInc * j)) {
                    if (exploredMap.clearForRobot(pos.y + rowInc * j, pos.x + colInc * j))
                        curDist++;
                        //Upon finding virtual wall; break
                    else
                        break;
                }
                //Upon reaching bordering walls; break
                else
                    break;
            }

            System.out.println("Direction: " + i + " " + curDist);
            //Evaluate closest distance for all direction and direction
            if (curDist < lowest) {
                lowest = curDist;
                lowestIter = i;
            }
        }
        System.out.println("Direction " + dir);
        //Identify direction using lowestIter (right,up,left,right)
        for (int c = 0; c < lowestIter; c++) {
            dir = Direction.getAntiClockwise(dir);
        }

        return dir;
    }

    /**
     * Process image string received from RPi by identifying position of image detected and direction of image
     * Then, store image position, direction and id in imageResult
     *
     * @param targetObsSurface
     * @param imgResult
     */
    private void processImgResult(ObsSurface targetObsSurface, String imgResult) {
        System.out.println("Processing image result");
        String imageId;
        JSONObject imageJSON;
        Direction obsDir;
        Point obsPos = new Point();
        //Need to identify: obstacle position, surface direction, surface id
        if (!imgResult.contains("None")) {
            try {
                String[] result = imgResult.split(",");
                System.out.println("Trying convert  number");
                //Leon fixed process integer
                int primitivePos = Integer.parseInt(result[0].trim());
                System.out.println("Grid position: " + primitivePos);
                imageId = result[1];
                System.out.println("Image Id: " + imageId);
                obsDir = targetObsSurface.getSurface();
                obsPos = processImgPosition(targetObsSurface, primitivePos);
                //TODO: Check if processing image position is accurate
//                imageJSON = "{x: " + obsPos.x + "," + obsPos.y + "|" + "Image Direction: " + obsDir.toString() +"|" + " Surface ID: " + imageId + "}\n";
                if(exploredMap.checkValidCell(obsPos.y, obsPos.x)){
                    System.out.println("nothingwrong");
                    System.out.printf("Reference cell - x:" + obsPos.x + " y:" + obsPos.y +"\n");
                    if(exploredMap.getCell(obsPos.y, obsPos.x).isObstacle() && !robot.checkIfCapturedImages(imageId)){
                        System.out.println("nothing wrong pls");
                        imageJSON = robot.getImageJSON(obsPos.x, obsPos.y, imageId, obsDir);
                        robot.getImageResult().put(imageJSON);
                        System.out.println(imageJSON.toString());
                        robot.setImageCount(robot.getImageCount()+1);
                        robot.addCapturedImages(imageId);
                    }
                }
            } catch (Exception e) {
                System.out.println(e);
            }
        }
    }

    private Point processImgPosition(ObsSurface obsSurface, int gridPos) {
        System.out.println("Processing image position");
        Point obsPos = new Point();
        int tempOffset;
        Direction dir = obsSurface.getSurface();
        if (gridPos == 2) {
            return obsSurface.getPos();
        }
        tempOffset = (gridPos - 2) * 1;
        obsPos.x = obsSurface.getPos().x;
        obsPos.y = obsSurface.getPos().y;

        switch (dir) {
            case UP:
                obsPos.x += tempOffset;
                break;
            case RIGHT:
                obsPos.y -= tempOffset;
                break;
            case DOWN:
                obsPos.x -= tempOffset;
                break;
            case LEFT:
                obsPos.y += tempOffset;
                break;
            default:
                break;
        }
        return obsPos;
    }

    public boolean imagePositionPossible(Map exploredMap, Direction obsSurfaceDir, Point robotPos) {
        Point targetPoint = new Point((robotPos.x + getColIncrementForMovement(obsSurfaceDir)), (robotPos.y + getRowIncrementForMovement(obsSurfaceDir)));
        if (exploredMap.checkValidCell(targetPoint.y, targetPoint.x)) {
            if (movable(obsSurfaceDir)) {
                return true;
            }
        }
        return false;
    }
}

