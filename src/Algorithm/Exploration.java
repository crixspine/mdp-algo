package Algorithm;

import Map.Map;
import Map.Cell;
import Map.Direction;
import Map.MapConstants;
import Map.ObsSurface;

import Network.NetMgr;
import Robot.Robot;
import Robot.Command;
import Robot.RobotConstants;
import org.json.JSONObject;

import java.awt.*;
import java.util.ArrayList;
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

    //Obtain row increment for robot's position for every direction
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

    //Exploration loop to move robot to new position using right wall hugging algorithm, and updates map in each iteration.
    //Exits exploration when coverage or time limit is reached, or when exploration is complete.
    public int exploration(Point start) throws InterruptedException {
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
                rightWallHugAlgo(false);
                if (robot.getPos().x == 1 && robot.getPos().y == 1) {
                    exploreMore = true;
                    areaExplored = exploredMap.getExploredPercentage();
                }

            } catch (InterruptedException e1) {
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
                    goToUnexploredCell2();
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
                    if (!goToUnexploredCell())
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
                rightWallHugAlgo(false);
                if (robot.getPos().x == 1 && robot.getPos().y == 1) {
                    exploreMore = true;
                    areaExplored = exploredMap.getExploredPercentage();
                }

            } catch (InterruptedException e1) {
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
                    goToUnexploredCell2();
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
                    if (!goToUnexploredCell())
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

        System.out.println("Starting to go to obstacle surfaces");
        goToObsSurfaces(exploredMap);
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

    public boolean goToObsSurfaces(Map exploredMap) throws InterruptedException {
        ObsSurface targetObsSurface;
        while (obsSurfaces.size() > 0) {
            System.out.print("No of surfaces left" + obsSurfaces.size());
            targetObsSurface = exploredMap.nearestObstacleSurface(robot.getPos(), obsSurfaces);
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


    private boolean goToUnexploredCell2() throws InterruptedException {
        robot.setStatus("Go to nearest unexplored\n");
        LOGGER.info(robot.getStatus());


        Cell nearestUnexplored = exploredMap.nearestUnexploredCell(robot.getPos());
        LOGGER.info("Nearest unexplored: " + nearestUnexplored);


        Cell nearestExp = exploredMap.nearestExploredCell(nearestUnexplored.getPos(), robot.getPos());
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

    //Robot move to nearest unexplored cell
    private boolean goToUnexploredCell() throws InterruptedException {
        robot.setStatus("Go to nearest unexplored\n");
        LOGGER.info(robot.getStatus());


        Cell nearestUnexplored = exploredMap.nearestUnexploredCell(robot.getPos());
        LOGGER.info("Nearest unexplored: " + nearestUnexplored);

        Cell nearestExp = exploredMap.nearestExploredCell(nearestUnexplored.getPos(), robot.getPos());
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


    //Execute command to capture image in front
    public void captureImageFront() throws InterruptedException {

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
            imgFileName = robot.takeImage();
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

    //Execute command to capture image on the right, i.e. turn right and capture image in front
    public void captureImageRight() throws InterruptedException {
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
            imgFileName = robot.takeImage();
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
    private void rightWallHugAlgo(boolean doingImage) throws InterruptedException {
//      //alignment at goal zone
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
        if (checkIfMovable(Direction.getClockwise(robotDir))) {

            //Execute image rec front if facing obstacle
            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                System.out.println("Testing for image position possible");
                if(imagePositionPossible(exploredMap, Direction.getOpposite(robot.getDir()),robot.getPos()))
                captureImageFront();
            }
            //On obstacle and steps count is more than zero-> outstanding
            if (!robot.isRightHuggingWall()) {
                robot.obstacleSide++;
                robot.obstacleStepsCounter = 0;

            }

            robot.turn(Command.TURN_RIGHT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            robot.sense(exploredMap, realMap);
            moveForward(RobotConstants.MOVE_STEPS, stepPerSecond, doingImage);
            right_move++;
            robot.obstacleStepsCounter++;
        }
        //Check if forward movement is possible
        else if (checkIfMovable(robotDir)) {

            robot.move(Command.FORWARD, RobotConstants.MOVE_STEPS, exploredMap, stepPerSecond);
            robot.sense(exploredMap, realMap);
            if (!robot.isRightHuggingWall()) {
                robot.obstacleStepsCounter++;
            }
            if (robot.obstacleStepsCounter == 3 && robot.isDoingImage() && robot.checkFrontForSingleObstacle(exploredMap, Direction.getClockwise(robot.getDir()))) {
                robot.align_front(exploredMap, realMap);
                captureImageRight();
            }
            right_move = 0;
        }

        //Check if can move in left direction
        else if (checkIfMovable(Direction.getAntiClockwise(robotDir))) {


            if (!sim) {
                if (robot.getAlignCount() == 0) {
                    robot.align_right(exploredMap, realMap);
                }
            }
            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                captureImageFront();
                robot.obstacleSide = 1;
            }
            if (!robot.isRightHuggingWall()) {
                robot.obstacleSide = 1;
                if (robot.obstacleStepsCounter > 0 && robot.isDoingImage()) {
                    robot.align_front(exploredMap, realMap);
                    captureImageRight();
                }
                robot.obstacleStepsCounter = 0;
            } else {
                robot.obstacleStepsCounter = 1;
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            robot.sense(exploredMap, realMap);

            moveForward(RobotConstants.MOVE_STEPS, stepPerSecond, doingImage);
            right_move = 0;

        }

        //If all fails, u-turn
        else {

            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                captureImageFront();
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            robot.sense(exploredMap, realMap);

            if (robot.align_front(exploredMap, realMap) && robot.isDoingImage() && !robot.isFacingWall()) {
                captureImageFront();
            }
            robot.turn(Command.TURN_LEFT, stepPerSecond);
            robot.setR1count(0);
            robot.setAlignCount(0);
            robot.sense(exploredMap, realMap);

        }
    }

    //Move forward in number of steps, sensing robot's environment after each step (if movable)
    private void moveForward(int steps, int stepPerSecond, boolean doingImage) throws InterruptedException {
        if (checkIfMovable(robot.getDir())) {       // for actual, double check in case of previous sensing error


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

    //Check if the next move in cells of that direction is a valid move
    private boolean checkIfMovable(Direction dir) {
        LOGGER.info("Checking if movable");
        int rowInc = getRowIncrementForMovement(dir);
        int colInc = getColIncrementForMovement(dir);

        return exploredMap.checkValidMove(robot.getPos().y + rowInc, robot.getPos().x + colInc);
    }

    //Check if robot and target location is at start position; rotates robot to face down if true
    private boolean robotAndTargetLocAtStartPos(Point loc) throws InterruptedException {
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

    //Execute commands generated by a star algorithm for fastest path to target location; senses and update map after
    private void executeCommandsToTargetLocation(ArrayList<Command> commands, Point loc) throws InterruptedException {
        for (Command c : commands) {
            System.out.println("Command: " + c);
            if ((c == Command.FORWARD) && !checkIfMovable(robot.getDir())) {
                System.out.println("Not Executing Forward Not Movable");
                // Recompute a star path to location
                goToPoint(loc);
                break;

            } else {
                //If last command is turn; robot has already reached the destination point
                if (((c == Command.TURN_LEFT && !checkIfMovable(Direction.getAntiClockwise(robot.getDir()))) ||
                        (c == Command.TURN_RIGHT && !checkIfMovable(Direction.getClockwise(robot.getDir())))) && commands.indexOf(c) == commands.size() - 1)
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

    //Moves robot to nearest virtual wall after reaching unexplored cell; continue right wall hugging algorithm upon
    private void continueExplorationUponNearestUnexplored() throws InterruptedException {
        robot.setStatus("Continue exploration, finding the nearest virtual wall.");
        LOGGER.info(robot.getStatus());

        //Get direction of the nearest virtual wall
        Direction dir = nearestVirtualWall(robot.getPos());
        System.out.println(dir);

        //If can move in the direction of nearest virtual wall, turn robot to face direction
        if (checkIfMovable(dir)) {
            while (dir != robot.getDir()) {
                if (dir.ordinal() - robot.getDir().ordinal() == 1)
                    robot.turn(Command.TURN_LEFT, stepPerSecond);
                else
                    robot.turn(Command.TURN_RIGHT, stepPerSecond);
            }

            //Keep moving in the same direction until unable (meet obstacle at the front)
            while (checkIfMovable(robot.getDir())) {
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

    //Execute commands generated by a star algorithm for fastest path to target location, senses and update map after
    private void executeCommandsToStartPoint(ArrayList<Command> commands, Point loc) throws InterruptedException {
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


    //Moves the robot to a specific point in the arena
    private boolean goToPoint(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetLocAtStartPos(loc)) {

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
                executeCommandsToTargetLocation(commands, loc);
                //Sense environment after movement
                //If robot moved to nearest unexplored area and still not finished exploration; find nearest virtual wall and continue exploration
                if (exploredMap.getExploredPercentage() < 100 && checkIfMovable(Direction.getClockwise(robot.getDir()))) {
                    continueExplorationUponNearestUnexplored();
                }
            }

            //Return to start position
            else {
                executeCommandsToStartPoint(commands, loc);
            }
        }
        //Robot successfully reached target location; return true
        return true;
    }

    //Moves the robot to a specific point in the arena
    private boolean goToPoint2(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetLocAtStartPos(loc)) {

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
                executeCommandsToTargetLocation(commands, loc);
                //Sense environment after movement
                //If robot moved to nearest unexplored area and still not finished exploration; find nearest virtual wall and continue exploration
            }

            //Return to start position
            else {
                executeCommandsToStartPoint(commands, loc);
            }
        }
        //Robot successfully reached target location; return true
        return true;
    }

    //Moves the robot to a specific point in the arena without sensing
    private boolean goToPointWithoutSensing(Point loc) throws InterruptedException {
        robot.setStatus("Go to point: " + loc.toString());
        LOGGER.info(robot.getStatus());
        if (!robotAndTargetLocAtStartPos(loc)) {

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
            executeCommandsToTargetLocation(commands, loc);

        }

        //Robot successfully reached target location; return true
        return true;
    }

    //Checks cell in up,down,left,right direction w.r.t robot in straight line; returns direction to nearest virtual wall
    //Returns the direction to the nearest virtual wall
    private Direction nearestVirtualWall(Point pos) {
        int rowInc, colInc, lowest = 1000, lowestIter = 0, curDist;
        //Priority of direction of wall; right, up, left, down
        Direction dir = Direction.RIGHT;
        //Evaluate the distance to nearest virtual wall
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

    //Process image string received from RPi by identifying position of image detected and direction of image
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
                if(exploredMap.checkValidCell(obsPos.y, obsPos.x)){
                    System.out.printf("Reference cell - x:" + obsPos.x + " y:" + obsPos.y +"\n");
                    if(exploredMap.getCell(obsPos.y, obsPos.x).isObstacle() && !robot.checkIfCapturedImages(imageId)){
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
            if (checkIfMovable(obsSurfaceDir)) {
                return true;
            }
        }
        return false;
    }
}