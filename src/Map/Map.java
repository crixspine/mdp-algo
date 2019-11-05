package Map;

import java.awt.Point;
import java.util.ArrayList;

public class Map {

    private final Cell[][] grid;
    private double exploredPercentage;

    public Map() {
        grid = new Cell[MapConstants.MAP_HEIGHT][MapConstants.MAP_WIDTH];
        initMap();
    }

    private void initMap() {
        // Init Cells on the grid
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                grid[row][col] = new Cell(new Point(col, row));

                // Init virtual wall
                if (row == 0 || col == 0 || row == MapConstants.MAP_HEIGHT - 1 || col == MapConstants.MAP_WIDTH - 1) {
                    grid[row][col].setVirtualWall(true);
                }
            }
        }
        exploredPercentage = 0.00;

    }

    public void resetMap() {
        initMap();
    }

    //Set the explored variable for all cells
    public void setAllExplored(boolean explored) {
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                grid[row][col].setExplored(explored);
            }
        }
        if (explored) {
            exploredPercentage = 100.00;
        } else {
            exploredPercentage = 0.00;
        }
    }

    //Set the moveThru variable for all cells
    public void setAllMoveThru(boolean moveThru) {
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                grid[row][col].setMoveThru(moveThru);
            }
        }
    }

    public double getExploredPercentage() {
        updateExploredPercentage();
        return this.exploredPercentage;
    }

    private void updateExploredPercentage() {
        double total = MapConstants.MAP_HEIGHT * MapConstants.MAP_WIDTH;
        double explored = 0;

        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                if (grid[row][col].isExplored())
                    explored++;
            }
        }

        this.exploredPercentage = explored / total * 100;
    }

    //Get cell using row and col
    public Cell getCell(int row, int col) {
        return grid[row][col];
    }

    //Get cell using Point(x, y)
    public Cell getCell(Point pos) {
        return grid[pos.y][pos.x];
    }

    //Check if the row and col is within the Map
    public boolean checkValidCell(int row, int col) {
        return row >= 0 && col >= 0 && row < MapConstants.MAP_HEIGHT && col < MapConstants.MAP_WIDTH;
    }

    //Check if movement can be made in the rol, col
    public boolean checkValidMove(int row, int col) {
        return checkValidCell(row, col) && !getCell(row, col).isVirtualWall() && !getCell(row, col).isObstacle() && getCell(row, col).isExplored();
    }

    //Set the moveThru para of the 3x3 grids moved through by the robot
    public void setPassThru(int row, int col) {
        for (int r = row - 1; r <= row + 1; r++) {
            for (int c = col - 1; c <= col + 1; c++) {
                grid[r][c].setMoveThru(true);
            }
        }
    }

    //Create new virtual wall around new found obstacles
    public void setVirtualWall(Cell obstacle, boolean isVirtualWall) {
        for (int r = obstacle.getPos().y - 1; r <= obstacle.getPos().y + 1; r++) {
            for (int c = obstacle.getPos().x - 1; c <= obstacle.getPos().x + 1; c++) {
                if (checkValidCell(r, c)) {
                    grid[r][c].setVirtualWall(isVirtualWall);
                }
            }
        }
    }


    //Get all movable neighbours Direction and Cell object
    public ArrayList<Cell> getNeighbours(Cell c) {

        ArrayList<Cell> neighbours = new ArrayList<Cell>();
        Point up = new Point(c.getPos().x, c.getPos().y + 1);
        Point down = new Point(c.getPos().x, c.getPos().y - 1);
        Point left = new Point(c.getPos().x - 1, c.getPos().y);
        Point right = new Point(c.getPos().x + 1, c.getPos().y);

        // UP
        if (checkValidMove(up.y, up.x)) {
            neighbours.add(getCell(up));
        }

        // DOWN
        if (checkValidMove(down.y, down.x)) {
            neighbours.add(getCell(down));
        }

        // LEFT
        if (checkValidMove(left.y, left.x)) {
            neighbours.add(getCell(left));
        }

        // RIGHT
        if (checkValidMove(right.y, right.x)) {
            neighbours.add(getCell(right));
        }

        return neighbours;
    }

    //Check if wayPoint is valid to move there cannot move to virtual wall
    public boolean wayPointClear(int row, int col) {
        return checkValidCell(row, col) && !getCell(row, col).isVirtualWall() && !getCell(row, col).isObstacle();
    }

    //Check whether a particular grid is clear for robot to move through
    public boolean clearForRobot(int row, int col) {
        for (int r = row - 1; r <= row + 1; r++) {
            for (int c = col - 1; c <= col + 1; c++) {
                if (!checkValidCell(r, c) || !grid[r][c].isExplored() || grid[r][c].isObstacle())
                    return false;
            }
        }
        return true;
    }

    //Return the nearest unexplored cell from a location
    public Cell nearestUnexploredCell(Point loc) {
        double dist = 1000, tempDist;
        Cell nearest = null, tempCell;

        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                tempCell = grid[row][col];
                tempDist = loc.distance(tempCell.getPos());
                if ((!tempCell.isExplored()) && (tempDist < dist)) {
                    nearest = tempCell;
                    dist = tempDist;
                }
            }
        }
        return nearest;
    }

    //Get all movable neighbours Direction and Cell object
    public Point getNeighbour(Point pos, Direction surfDir) {

        Point n = null;

        switch (surfDir) {
            case UP:
                n = new Point(pos.x, pos.y + 1);
                break;
            case DOWN:
                n = new Point(pos.x, pos.y - 1);
                break;
            case LEFT:
                n = new Point(pos.x - 1, pos.y);
                break;
            case RIGHT:
                n = new Point(pos.x + 1, pos.y);
                break;
        }
        return n;
    }

    //Return the nearest obstacle surface
    public ObsSurface nearestObstacleSurface(Point loc, ArrayList<ObsSurface> notYetTaken) {
        double dist = 1000, tempDist;
        Point tempPos;
        ObsSurface nearest = null;

        for (ObsSurface obstacle : notYetTaken) {
//            tempPos = obstacle.getPos();
            // neighbour cell of that surface
            tempPos = getNeighbour(obstacle.getPos(), obstacle.getSurface());
            tempDist = loc.distance(tempPos);
            if (tempDist < dist) {
                dist = tempDist;
                nearest = obstacle;
            }
        }
        return nearest;
    }

    //Return the nearest explored but not move through cell given the nearest unexplored cell
    public Cell nearestExploredCell(Point loc, Point botLoc) {
        Cell cell, nearest = null;
        double distance = 1000;
        double botDistance = 1000;

        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                cell = grid[row][col];
                if (checkValidMove(row, col) && clearForRobot(row, col) && areaMoveThru(row, col)) {
                    if ((distance > loc.distance(cell.getPos()) && cell.getPos().distance(botLoc) < botDistance)) {       // actually no need to check for botLoc
                        nearest = cell;
                        distance = loc.distance(cell.getPos());
                        botDistance = cell.getPos().distance(botLoc);
                    }
                }
            }
        }
        System.out.println(nearest);
        return nearest;
    }

    //Check whether the entire area was moved through by the robot
    public boolean areaMoveThru(int row, int col) {
        for (int r = row - 1; r <= row + 1; r++) {
            for (int c = col - 1; c <= col + 1; c++) {
                if (!grid[r][c].isMoveThru()) {
                    return true;
                }
            }
        }
        return false;
    }

    //Remove existing cell with path
    public void removePaths() {
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                grid[r][c].setPath(false);
            }
        }
    }

    //Get the moving direction from point A to point B. (provided A or B has same x or y)
    public Direction getCellDir(Point A, Point B) {
        if (A.y - B.y > 0) {
            return Direction.DOWN;
        } else if (A.y - B.y < 0) {
            return Direction.UP;
        } else if (A.x - B.x > 0) {
            return Direction.LEFT;
        } else {
            return Direction.RIGHT;
        }
    }

    //reinit virtual wall when removing phantom blocks
    public void reinitializeVirtualWall() {
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                // Init Virtual wall
                if (row == 0 || col == 0 || row == MapConstants.MAP_HEIGHT - 1 || col == MapConstants.MAP_WIDTH - 1) {
                    grid[row][col].setVirtualWall(true);
                }
                if (grid[row][col].isObstacle()) {
                    for (int r = row - 1; r <= row + 1; r++)
                        for (int c = col - 1; c <= col + 1; c++)
                            if (checkValidCell(r, c))
                                grid[r][c].setVirtualWall(true);
                }
            }
        }
    }
}
