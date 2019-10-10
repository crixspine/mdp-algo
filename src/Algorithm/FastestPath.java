package Algorithm;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.logging.Logger;

import Map.*;
import Robot.Robot;
import Robot.Command;
import Robot.RobotConstants;

import static java.lang.Math.*;

public class FastestPath {

    private static final Logger LOGGER = Logger.getLogger(FastestPath.class.getName());

    private boolean sim;
    private Map exploredMap; //map after exploration
    private Robot robot;
    private HashMap<Point, Double> gCostMap; //map of g cost for every cell
    private HashMap<Cell,Cell> pathMap = new HashMap<Cell, Cell>();

    public FastestPath(Map exploredMap, Robot robot, boolean sim) {
        this.exploredMap = exploredMap;
        this.robot = robot;
        this.sim = sim;
        initCostMap();
    }

    public void initCostMap() {
        gCostMap = new HashMap<Point, Double>();
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row ++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col ++) {
                Cell cell = exploredMap.getCell(row, col);
                if (cell.movableCell()) { // Cell is movable if explored, and not obstacle or virtual wall
                    gCostMap.put(cell.getPos(), 0.0); //init costs of all cells as 0
                }
                else {
                    gCostMap.put(cell.getPos(), RobotConstants.INFINITE_COST); //if cell not movable set infinite to avoid
                }
            }
        }
    }

    public ArrayList<Cell> runAStar(Point start, Point goal, Direction initDir) {
        ArrayList<Cell> openList = new ArrayList<Cell>(); //not yet visited cells; "open" list
        ArrayList<Cell> closedList = new ArrayList<Cell>(); //visited cells; "closed" list
        ArrayList<Cell> neighbours; //neighbouring cells list
        double newGtemp, curGtemp;

        // init
        String status = String.format("Finding fastest path from %s to %s, initial direction: %s", start.toString().substring(14), goal.toString().substring(14), initDir.toString());
        robot.setStatus(status);
        LOGGER.info(status);
        Cell curCell = exploredMap.getCell(start);
        openList.add(curCell);
        Direction curDir = initDir;

        while (!openList.isEmpty()) { //while there are cells not yet visited
            curCell = getMinCostCell(openList, goal); //get cell with lowest cost in open list
            if (pathMap.containsKey(curCell)) {
                curDir = exploredMap.getCellDir(pathMap.get(curCell).getPos(), curCell.getPos());
            }
            closedList.add(curCell);
            openList.remove(curCell);
            if (closedList.contains(exploredMap.getCell(goal))) { //if goal is reached
                LOGGER.info("Fastest path found");
                return getPath(start, goal);
            }
            else {
                neighbours = exploredMap.getNeighbours(curCell);
                for (Cell n: neighbours) {
                    if (closedList.contains(n)) { //if neighbour already visited, ignore
                        continue;
                    }
                    else {
                        newGtemp = gCostMap.get(curCell.getPos()) + calculateG(curCell.getPos(), n.getPos(), curDir); //calculate new G value
                        if (openList.contains(n)) { //if current cell already in open list, i.e. was a neighbour of a previous cell before
                            curGtemp = gCostMap.get(n.getPos());
                            if (newGtemp < curGtemp) {
                                gCostMap.replace(n.getPos(), newGtemp); //update the G value if the new path to current cell has lower G value
                                pathMap.replace(n, curCell); //update path with lower G value
                            }
                        }
                        else { //if neighbour not yet visited
                            pathMap.put(n, curCell); //set new path to current cell
                            gCostMap.put(n.getPos(), newGtemp); //set new G value of current cell
                            openList.add(n); //add current cell to open list, i.e. it has been a neighbour cell
                        }
                    }
                }
            }
        }
        LOGGER.warning(String.format("Cannot find a fastest path from %s to %s, dir: %s", start.toString().substring(14), goal.toString().substring(14), initDir.toString()));
        return null;
    }

    //Returns the path from the prevCell hashmap, moving backwards from goal to start
    public ArrayList<Cell> getPath(Point start, Point goal) {
        Cell curCell = exploredMap.getCell(goal); //set current cell as goal
        Cell startCell = exploredMap.getCell(start);
        ArrayList<Cell> path = new ArrayList<Cell>();
        while(curCell != startCell) { //while current cell is not start cell
            path.add(curCell); //add current cell to final path
            curCell = pathMap.get(curCell); //iterate until start cell is reached
        }
        Collections.reverse(path); //reverse the goal -> start to make it start -> goal
        System.out.println(path);
        return path;
    }

    //To display fastest path on simulator
    public void displayFastestPath(ArrayList<Cell> path, boolean display) {
        Cell temp;
        System.out.println("Path:");
        for(int i = 0; i < path.size(); i++) {
            temp = path.get(i);
            //Set the path cells to display as path on the Sim
            exploredMap.getCell(temp.getPos()).setPath(display);
            System.out.println(exploredMap.getCell(temp.getPos()).toString());

            //Output Path on console
            if(i != (path.size()-1)) {
                System.out.print("(" + temp.getPos().y + ", " + temp.getPos().x + ") --> ");
            }
            else {
                System.out.print("(" + temp.getPos().y + ", " + temp.getPos().x + ")");
            }
        }
        System.out.println("\n");
    }

    //Returns the movements required to execute the path
    //MODIFY??
    public ArrayList<Command> getPathCommands(ArrayList<Cell> path) throws InterruptedException {
        Robot tempRobot = new Robot(true, true, robot.getPos().y, robot.getPos().x, robot.getDir()); //sim:false for actual run?
        ArrayList<Command> moves = new ArrayList<Command>();

        Command move;
        Cell cell = exploredMap.getCell(tempRobot.getPos());
        Cell newCell;
        Direction cellDir;

        //Iterate through the path
        for (int i = 0; i < path.size(); i++) {
            newCell = path.get(i);
            cellDir = exploredMap.getCellDir(cell.getPos(), newCell.getPos());
            // If the TempRobot and cell direction not the same
            if (Direction.getOpposite(tempRobot.getDir()) == cellDir) {
//                // 1. use backwards
//                move = Command.BACKWARD;
                move = Command.TURN_LEFT; //first move
                tempRobot.turn(move, RobotConstants.STEP_PER_SECOND);
                moves.add(move);
                tempRobot.turn(move, RobotConstants.STEP_PER_SECOND);
                moves.add(move);
                move = Command.FORWARD; //second move

            } else if (Direction.getClockwise(tempRobot.getDir()) == cellDir) {
                move = Command.TURN_RIGHT; //first move
                tempRobot.turn(move, RobotConstants.STEP_PER_SECOND);
                moves.add(move); //second move
                move = Command.FORWARD;
            } else if (Direction.getAntiClockwise(tempRobot.getDir()) == cellDir) {
                move = Command.TURN_LEFT; //first move
                tempRobot.turn(move, RobotConstants.STEP_PER_SECOND);
                moves.add(move);
                move = Command.FORWARD; //second move
            } else {
                move = Command.FORWARD;
            }
            tempRobot.move(move, RobotConstants.MOVE_STEPS, exploredMap, RobotConstants.STEP_PER_SECOND);
            moves.add(move);
            cell = newCell;
        }
        System.out.println("Generated Moves: " + moves.toString());
        return moves;
    }

    //To get cell with the lowest F cost in the open list; F = G + H
    private Cell getMinCostCell(ArrayList<Cell> openList, Point goal) {
        Cell cell = null;
        Point pos;
        double minCost = RobotConstants.INFINITE_COST;

        for (Cell cellTemp : openList) {
            pos = cellTemp.getPos();
            double fCost = gCostMap.get(pos) + calculateH(pos, goal);
            if(fCost < minCost) {
                minCost = fCost;
                cell = cellTemp;
            }
        }
        return cell;
    }

    // Calculate G cost from point A to point B in a given direction
    private double calculateG(Point A, Point B, Direction dir) {
        return calculateMoveCost(A, B) + calculateTurnCost(dir, exploredMap.getCellDir(A, B));
    }

    //Calculate H cost (heuristics) from a point to the goal; using Manhattan distance
    private double calculateH(Point pt, Point goal) {
        return pt.distance(goal);
    }

    //Calculate cost to move from point A to point B
    private double calculateMoveCost(Point A, Point B) {
        double steps =  abs(A.x - B.x) + abs(A.y - B.y);
        return RobotConstants.MOVE_COST * steps;
    }

    //calculate cost to turn from direction A to direction B
    private double calculateTurnCost(Direction dirA, Direction dirB) {

        //Max of 2 turns in either direction, same direction will get 0
        int turns = abs(dirA.ordinal() - dirB.ordinal());

        if(turns > 2) {
            turns %= 2;
        }
        return turns * RobotConstants.TURN_COST;
    }

}
