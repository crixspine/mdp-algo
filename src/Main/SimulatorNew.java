package Main;

import Algorithm.Exploration;
import Algorithm.FastestPath;
import Map.Cell;
import Map.*;
import Network.NetMgr;
import Network.NetworkConstants;
import Robot.Command;
import Robot.Robot;
import Robot.RobotConstants;
import Robot.Sensor;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.concurrent.Task;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.*;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.TextAlignment;
import javafx.stage.*;
import javafx.animation.AnimationTimer;

import java.awt.*;
import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

import static java.lang.Math.abs;
import Timer.*;

//JavaFX Libraries

public class SimulatorNew extends Application {

    private static final Logger LOGGER = Logger.getLogger(SimulatorNew.class.getName());

    // Program Variables
    private Map map; // Used to hold loaded Map for sim
    private Map exploredMap;
    //fastestPathMap;
    private Map newExploredMap;
    private Point wayPoint = new Point(MapConstants.GOALZONE_COL, MapConstants.GOALZONE_ROW);
    private Point startPos = new Point(1, 1);
    private Robot robot;
    private boolean sim = true;
    private boolean expMapDraw = true;
    private boolean newExpMapDraw = true;
    private AnimationTimer animateTimer1;
    private AnimationTimer animateTimer2;
    public static DisplayTimer displayTimer = new DisplayTimer();

    private MapDescriptor mapDescriptor = new MapDescriptor();
    private String defaultMapPath = "defaultMap.txt";

    private static final NetMgr netMgr = NetMgr.getInstance();

    private boolean setWaypoint = false;
    private boolean setRobot = false;

    // Mode Constants
    private final String SIM = "Simulation";
    private final String REAL = "Actual Run";
    private final String FASTEST_PATH = "Fastest Path";
    private final String EXPLORATION = "Exploration";
    private final String IMAGE = "Image Rec";
    private final int MAX_WIDTH = 1000;

    // initial task set to exploration
    private String taskSelected = EXPLORATION;

    // GUI Components
    private Canvas mapGrid;
    private GraphicsContext gc;
    private Scene dialogScene;
    private Stage dialog;

    private Canvas newMapGrid;
    private GraphicsContext newGC;

    // UI components
    private Button loadMapBtn, newMapBtn, saveMapBtn, resetMapBtn, startBtn, connectBtn, setWaypointBtn, setRobotBtn,
            setObstacleBtn, cancelBtn, confirmBtn;
    private RadioButton expRB, fastPathRB, imageRB, simRB, realRB, upRB, downRB, leftRB, rightRB;
    private ToggleGroup mode, task, startDir;
    private TextArea debugOutput, imageOutput;
    private ScrollBar timeLimitSB, coverageLimitSB, stepsSB;
    private TextField startPosTxt, wayPointTxt, timeLimitTxt, coverageLimitTxt, stepsTxt, mapTxt;
    private Label genSetLbl, simSetLbl, arenaSetLbl, startPosLbl, startDirLbl, wayPointLbl, timeLimitLbl, coverageLimitLbl, stepsLbl;
    private Label modeChoiceLbl, taskChoiceLbl, mapChoiceLbl, statusLbl, timerLbl, mapLbl;
    private Label timerTextLbl;
    private FileChooser fileChooser;

    // Threads for each of the tasks
    private Thread fastTask, expTask, imageTask;
    private boolean taskStarted = false, taskPaused = false;
    private Thread startedTask = null;

    public void start(Stage primaryStage) {
        // Init for Map and Robot
        map = new Map();
        newExploredMap = new Map();
        // Set to all explored for loading and saving Map
        map.setAllExplored(true);
        exploredMap = new Map();


        // Default Location at the startzone
        robot = new Robot(sim, false, 1, 1, Direction.RIGHT);
        robot.setStartPos(robot.getPos().y, robot.getPos().x, exploredMap);

        // Threads

        // Setting the Title and Values for the Window
        primaryStage.setTitle("AY 2019/2020 Semester 1 MDP Group 4");
        GridPane grid = new GridPane();
        GridPane controlGrid = new GridPane();
        GridPane debugGrid = new GridPane();
        // Grid Settings
        grid.setAlignment(Pos.CENTER);
        grid.setHgap(5);
        grid.setVgap(5);
        grid.setPadding(new Insets(5, 5, 5, 5));

        controlGrid.setAlignment(Pos.CENTER);
        controlGrid.setHgap(5);
        controlGrid.setVgap(5);

        // Drawing Component
        mapGrid = new Canvas(MapConstants.MAP_CELL_SZ * MapConstants.MAP_WIDTH + 1 + MapConstants.MAP_OFFSET,
                MapConstants.MAP_CELL_SZ * MapConstants.MAP_HEIGHT + 1 + MapConstants.MAP_OFFSET);
        gc = mapGrid.getGraphicsContext2D();

        animateTimer1 = new AnimationTimer() {

            private long startTime ;

            @Override
            public void start() {
                startTime = System.currentTimeMillis();
                super.start();
            }

            @Override
            public void handle(long timestamp) {
                long now = System.currentTimeMillis();
                drawMap(expMapDraw);
                drawRobot();
                debugOutput.setText(robot.getStatus() + "\n" + robot.toString());
                imageOutput.setText(robot.getImageResult().toString());
                //mapOutput.setText("hello");

                timerTextLbl.setText(displayTimer.getTimerLbl());
                if (startedTask != null) {
                    if (!startedTask.isAlive()) {
                        startBtn.setVisible(false);
                    }
                }
            }
        };
        animateTimer2 = new AnimationTimer() {

            private long startTime ;

            @Override
            public void start() {
                startTime = System.currentTimeMillis();
                super.start();
            }

            @Override
            public void handle(long timestamp) {
                long now = System.currentTimeMillis();
                drawNewMap(false);
            }
        };

        animateTimer1.start();

        // Canvas MouseEvent
        mapGrid.setOnMouseClicked(MapClick);
        mapGrid.setUserData("Main_Map");

        // Lbl Init
        genSetLbl = new Label("General Settings");
        arenaSetLbl = new Label("Arena Settings");
        simSetLbl = new Label("Simulator Settings");
        genSetLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        arenaSetLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        simSetLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        genSetLbl.setMaxWidth(MAX_WIDTH);
        arenaSetLbl.setMaxWidth(MAX_WIDTH);
        simSetLbl.setMaxWidth(MAX_WIDTH);
        startPosLbl = new Label("Start Position: ");
        startDirLbl = new Label("Start Direction: ");
        startPosTxt = new TextField();
        startPosTxt.setText(String.format("(%d, %d)", robot.getPos().x, robot.getPos().y));
        startPosTxt.setMaxWidth(MAX_WIDTH);
        startPosTxt.setDisable(true);
        wayPointLbl = new Label("Way Point:");
        wayPointTxt = new TextField();
        wayPointTxt.setText(String.format("(%d, %d)", wayPoint.x, wayPoint.y));
        wayPointTxt.setMaxWidth(MAX_WIDTH);
        wayPointTxt.setDisable(true);
        timeLimitLbl = new Label("Time Limit: ");
        coverageLimitLbl = new Label("Coverage Limit:");
        timeLimitTxt = new TextField();
        coverageLimitTxt = new TextField();
        modeChoiceLbl = new Label("Mode:");
        taskChoiceLbl = new Label("Task:");
        timeLimitTxt.setDisable(true);
        coverageLimitTxt.setDisable(true);
        stepsLbl = new Label("Steps: ");
        stepsTxt = new TextField();
        stepsTxt.setDisable(true);
        stepsTxt.setMaxWidth(100);
        timeLimitTxt.setMaxWidth(100);
        coverageLimitTxt.setMaxWidth(100);

        mapChoiceLbl = new Label("Map File: ");
        mapTxt = new TextField();
        mapTxt.setText(defaultMapPath);
        mapTxt.setDisable(true);
        mapTxt.setMaxWidth(MAX_WIDTH);

        statusLbl = new Label("Robot Status");
        statusLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        statusLbl.setMaxWidth(MAX_WIDTH);

        timerLbl = new Label("Timer");
        timerLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        timerLbl.setMaxWidth(MAX_WIDTH);

        mapLbl = new Label("Map Descriptor");
        mapLbl.setBackground(new Background(new BackgroundFill(Color.BURLYWOOD, CornerRadii.EMPTY, Insets.EMPTY)));
        mapLbl.setMaxWidth(MAX_WIDTH);

        // Buttons Init
        connectBtn = new Button("Connect");
        startBtn = new Button("Start");
        loadMapBtn = new Button("Load Map");
        newMapBtn = new Button("New Map");
        saveMapBtn = new Button("Save Map");
        resetMapBtn = new Button("Reset Map");
        setWaypointBtn = new Button("Reset Waypoint");
        setWaypointBtn.setMaxWidth(MAX_WIDTH);
        setRobotBtn = new Button("Reset Starting Position");
        setRobotBtn.setMaxWidth(MAX_WIDTH);
        setObstacleBtn = new Button("Set Obstacles");
        cancelBtn = new Button("Cancel");
        cancelBtn.setMaxWidth(MAX_WIDTH);
        confirmBtn = new Button("Confirm");
        confirmBtn.setMaxWidth(MAX_WIDTH);

        loadMapBtn.setMaxWidth(MAX_WIDTH);
        saveMapBtn.setMaxWidth(MAX_WIDTH);
        newMapBtn.setMaxWidth(MAX_WIDTH);

        // Radio Buttom Init
        expRB = new RadioButton(EXPLORATION);
        fastPathRB = new RadioButton(FASTEST_PATH);
        imageRB = new RadioButton(IMAGE);
        simRB = new RadioButton(SIM);
        realRB = new RadioButton(REAL);
        upRB = new RadioButton("UP");
        downRB = new RadioButton("DOWN");
        leftRB = new RadioButton("LEFT");
        rightRB = new RadioButton("RIGHT");


        // Toggle Group Init
        mode = new ToggleGroup();
        simRB.setToggleGroup(mode);
        realRB.setToggleGroup(mode);
        simRB.setSelected(true);

        mode.selectedToggleProperty().addListener(new ChangeListener<Toggle>() {
            @Override
            public void changed(ObservableValue<? extends Toggle> observable, Toggle oldValue, Toggle newValue) {
                if (simRB.isSelected()) {
                    sim = true;
                }
                else {
                    System.out.println("Actual run selected, connecting to RPI");
                    netMgr.initConn();
                    sim = false;
                    stepsSB.setValue(30);   // set to max to avoid any delay
                }
            }
        });

        task = new ToggleGroup();
        expRB.setToggleGroup(task);
        expRB.setSelected(true);
        fastPathRB.setToggleGroup(task);
        imageRB.setToggleGroup(task);

        task.selectedToggleProperty().addListener(new ChangeListener<Toggle>() {
            @Override
            public void changed(ObservableValue<? extends Toggle> observable, Toggle oldValue, Toggle newValue) {
                if (simRB.isSelected() && fastPathRB.isSelected()) {
                    internalHandleResetMap();
                }
                if (simRB.isSelected() && expRB.isSelected()) {
                    internalHandleResetMap();
                }
                if (simRB.isSelected() && imageRB.isSelected()) {
                    internalHandleResetMap();
                }
                if (expRB.isSelected()) {
                    taskSelected = EXPLORATION;
                } else if (fastPathRB.isSelected()) {
                    taskSelected = FASTEST_PATH;
                } else if (imageRB.isSelected()) {
                    taskSelected = IMAGE;
                }
            }
        });

        startDir = new ToggleGroup();
        upRB.setToggleGroup(startDir);
        downRB.setToggleGroup(startDir);
        leftRB.setToggleGroup(startDir);
        rightRB.setToggleGroup(startDir);
        rightRB.setSelected(true);
        startDir.selectedToggleProperty().addListener(new ChangeListener<Toggle>() {
            @Override
            public void changed(ObservableValue<? extends Toggle> observable, Toggle oldValue, Toggle newValue) {
                RadioButton button = (RadioButton) startDir.getSelectedToggle();
                Direction newDir = Direction.valueOf(button.getText());
                try {
                    if (Direction.getAntiClockwise(robot.getDir()) == newDir) {
                        robot.turn(Command.TURN_LEFT, RobotConstants.STEP_PER_SECOND);
                    } else if (Direction.getClockwise(robot.getDir()) == newDir) {
                        robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
                    } else {
                        robot.turn(Command.TURN_LEFT, RobotConstants.STEP_PER_SECOND);
                        robot.turn(Command.TURN_LEFT, RobotConstants.STEP_PER_SECOND);
                    }
                    System.out.println("Setting robot to new direction: " + newDir.toString() + "\n");
                } catch (InterruptedException e) {
                    LOGGER.warning("InterruptException");
                    e.printStackTrace();
                }
            }
        });

        // TextArea
        debugOutput = new TextArea();
        debugOutput.setMaxHeight(100);
        imageOutput = new TextArea();
        imageOutput.setMaxHeight(100);

        // File Chooser
        fileChooser = new FileChooser();

        // ScrollBar
        timeLimitSB = new ScrollBar();
        coverageLimitSB = new ScrollBar();
        stepsSB = new ScrollBar();
        stepsSB.setMin(1);
        stepsSB.setMax(30);
        timeLimitSB.setMin(10);
        timeLimitSB.setMax(240);
        coverageLimitSB.setMin(10);
        coverageLimitSB.setMax(100);

        connectBtn.setMaxWidth(MAX_WIDTH);
        startBtn.setMaxWidth(MAX_WIDTH);
        loadMapBtn.setMaxWidth(MAX_WIDTH);
        resetMapBtn.setMaxWidth(MAX_WIDTH);

        //load default variables
        double coverageLimit = 100;
        int timeLimit = 240000;
        int steps = 5;

        coverageLimitTxt.setText("" + (int) coverageLimit + " %");
        coverageLimitSB.setValue(coverageLimit);

        timeLimitTxt.setText("" + timeLimit / 1000 + " s");
        timeLimitSB.setValue(timeLimit);

        stepsTxt.setText("" + steps + " steps per seconds");
        stepsSB.setValue(steps);

        // load default map from defaultMapPath
        mapDescriptor.loadRealMap(map, defaultMapPath);
        mapDescriptor.loadRealMap(exploredMap, defaultMapPath);     // to display when start the app

        // Button ActionListeners
        resetMapBtn.setOnMouseClicked(resetMapBtnClick);
        startBtn.setOnMouseClicked(startBtnClick);    // to be uncommented after the class is uncommented
        setRobotBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                exploredMap.resetMap();
                mapDescriptor.loadRealMap(exploredMap, defaultMapPath);
                setRobot = !setRobot;
                if (!setRobot)
                    setRobotBtn.setText("Reset Starting Position");
                else
                    setRobotBtn.setText("Confirm Starting Position");

                setWaypoint = false;
            }
        });
        newMapBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent e) {
                newExploredMap.resetMap();
                dialog = new Stage();
                dialog.initModality(Modality.APPLICATION_MODAL);
                dialog.initOwner(primaryStage);

                newMapGrid = new Canvas(MapConstants.MAP_CELL_SZ * MapConstants.MAP_WIDTH + 1 + MapConstants.MAP_OFFSET,
                        MapConstants.MAP_CELL_SZ * MapConstants.MAP_HEIGHT + 1 + MapConstants.MAP_OFFSET);
                newGC = newMapGrid.getGraphicsContext2D();

                // Grid Settings for new map
                GridPane newGridForMap = new GridPane();
                GridPane buttonGrid = new GridPane();
                buttonGrid.setAlignment(Pos.CENTER);
                buttonGrid.setHgap(5);
                buttonGrid.setVgap(5);
                newGridForMap.setAlignment(Pos.CENTER);
                newGridForMap.setHgap(5);
                newGridForMap.setVgap(5);
                newGridForMap.setPadding(new Insets(5, 5, 5, 5));


                VBox vBox = new VBox();
                vBox.setPrefWidth(100);

                cancelBtn.setMinWidth(vBox.getPrefWidth());
                confirmBtn.setMinWidth(vBox.getPrefWidth());

                vBox.getChildren().addAll(cancelBtn, confirmBtn);

                buttonGrid.add(cancelBtn, 3, 1);
                buttonGrid.add(confirmBtn, 4, 1);

                ColumnConstraints col1 = new ColumnConstraints();
                col1.setPercentWidth(0);
                ColumnConstraints col2 = new ColumnConstraints();
                col2.setPercentWidth(10);
                buttonGrid.getColumnConstraints().setAll(col1, col2);

                newGridForMap.add(newMapGrid, 0, 0);
                newGridForMap.add(buttonGrid, 0, 1);
                dialogScene = new Scene(newGridForMap, 600, 600);


                setRobot = false;
                setWaypoint = false;
                Boolean alreadyExplored = false;

                animateTimer1.stop();
                animateTimer2.start();

                // Canvas MouseEvent
                newMapGrid.setOnMouseClicked(NewMapClick);
                newMapGrid.setUserData("New_Map");
                dialog.setScene(dialogScene);
                dialog.show();
            }
        });
        setWaypointBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                exploredMap.resetMap();
                mapDescriptor.loadRealMap(exploredMap, defaultMapPath);
                setWaypoint = !setWaypoint;
                if(setWaypoint)
                    setWaypointBtn.setText("Confirm Waypoint");
                else
                    setWaypointBtn.setText("Reset Waypoint");
                setRobot = false;
            }
        });
        cancelBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                dialog.close();
                animateTimer2.stop();
                animateTimer1.start();
            }
        });
        confirmBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                newExploredMap.setAllExplored(true);
                mapDescriptor.saveRealMap(newExploredMap, defaultMapPath);
                map.resetMap();
                exploredMap.resetMap();
                mapDescriptor.loadRealMap(map, defaultMapPath);
                mapDescriptor.loadRealMap(exploredMap, defaultMapPath);
                dialog.close();
                animateTimer2.stop();
                animateTimer1.start();
            }
        });

        loadMapBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                fileChooser.setTitle("Choose file to load Map from");
                File file = fileChooser.showOpenDialog(primaryStage);
                if (file != null) {
                    map.resetMap();
                    exploredMap.resetMap();
                    mapDescriptor.loadRealMap(map, file.getAbsolutePath());
                    mapTxt.setText(file.getName());
                    mapDescriptor.saveRealMap(map, defaultMapPath);
                    mapDescriptor.loadRealMap(exploredMap, defaultMapPath);
                }
                expMapDraw = true;
            }
        });
        saveMapBtn.setOnMouseClicked(new EventHandler<MouseEvent>() {
            public void handle(MouseEvent e) {
                fileChooser.setTitle("Choose file to save Map to");
                File file = fileChooser.showOpenDialog(primaryStage);
                if (file != null) {
                    mapDescriptor.saveRealMap(map, file.getAbsolutePath());
                }
            }
        });

        timeLimitSB.valueProperty().addListener(change -> {
            timeLimitTxt.setText("" + (int) timeLimitSB.getValue() + " s");
        });

        coverageLimitSB.valueProperty().addListener(change -> {
            coverageLimitTxt.setText("" + (int) coverageLimitSB.getValue() + "%");
        });

        stepsSB.valueProperty().addListener(change -> {
            stepsTxt.setText("" + (int) stepsSB.getValue() + " steps per second");
        });

        // TIMER
        timerTextLbl = new Label();
        timerTextLbl.setTextFill(Color.BLACK);
        timerTextLbl.setStyle("-fx-font-size: 3em;");
        timerTextLbl.setMaxWidth(MAX_WIDTH);
        timerTextLbl.setTextAlignment(TextAlignment.LEFT);
        timerTextLbl.setAlignment(Pos.CENTER);
        timerTextLbl.setText(displayTimer.getTimerLbl());

        controlGrid.add(genSetLbl, 0, 0, 4, 1);
        controlGrid.add(modeChoiceLbl, 0, 1);
        controlGrid.add(simRB, 1, 1);
        controlGrid.add(realRB, 2, 1);

        controlGrid.add(taskChoiceLbl, 0, 2);
        controlGrid.add(expRB, 1, 2);
        controlGrid.add(fastPathRB, 2, 2);
        controlGrid.add(imageRB, 3, 2);
        controlGrid.add(startBtn, 0, 3, 4,1 );

        controlGrid.add(arenaSetLbl, 0, 4, 4, 1);

        controlGrid.add(startPosLbl, 0, 5);
        controlGrid.add(startPosTxt, 1, 5, 1, 1);
        controlGrid.add(setRobotBtn, 2, 5, 2, 1);

        controlGrid.add(startDirLbl, 0, 6);
        controlGrid.add(upRB, 1, 6);
        controlGrid.add(downRB, 2, 6);
        controlGrid.add(leftRB, 1, 7);
        controlGrid.add(rightRB, 2, 7);

        controlGrid.add(wayPointLbl, 0, 8);
        controlGrid.add(wayPointTxt, 1, 8, 1, 1);
        controlGrid.add(setWaypointBtn, 2, 8, 2, 1);

        controlGrid.add(simSetLbl, 0, 9, 4, 1);

        controlGrid.add(timeLimitLbl, 0, 10, 1, 1);
        controlGrid.add(timeLimitSB, 1, 10, 2, 1);
        controlGrid.add(timeLimitTxt, 3, 10, 1, 1);

        controlGrid.add(coverageLimitLbl, 0, 11, 1, 1);
        controlGrid.add(coverageLimitSB, 1, 11, 2, 1);
        controlGrid.add(coverageLimitTxt, 3, 11, 1, 1);

        controlGrid.add(stepsLbl, 0, 12, 1, 1);
        controlGrid.add(stepsSB, 1, 12, 2, 1);
        controlGrid.add(stepsTxt, 3, 12, 1, 1);

        controlGrid.add(mapChoiceLbl, 0, 13);
        controlGrid.add(mapTxt, 1, 13);
        controlGrid.add(loadMapBtn, 2, 13,2,1);
        controlGrid.add(newMapBtn, 0, 14,2,1);
        controlGrid.add(saveMapBtn, 2, 14,2,1);

        controlGrid.add(resetMapBtn, 0, 15, 4, 1);

        controlGrid.add(statusLbl, 0, 16, 2, 1);
        controlGrid.add(debugOutput, 0, 17, 2, 1);

        controlGrid.add(mapLbl, 2, 16, 1, 1);
        controlGrid.add(imageOutput, 2, 17, 1, 1);

        controlGrid.add(timerLbl, 3, 16, 1, 1);
        controlGrid.add(timerTextLbl, 3, 17, 1, 1);

        ColumnConstraints col1 = new ColumnConstraints();
        col1.setPercentWidth(40);
        ColumnConstraints col2 = new ColumnConstraints();
        col2.setPercentWidth(60);
        ColumnConstraints col3 = new ColumnConstraints();
        col3.setPercentWidth(25);
        grid.getColumnConstraints().setAll(col1, col2);
        controlGrid.getColumnConstraints().setAll(col3, col3, col3, col3);

        grid.add(mapGrid, 0, 0);
        grid.add(controlGrid, 1, 0);

        // Dimensions of the Window
        Scene scene = new Scene(grid, 1000, 600);
        primaryStage.setScene(scene);
        scene.setOnKeyPressed(new EventHandler<KeyEvent>() {
            public void handle(KeyEvent e) {
                sim = false;
                robot.setSim(sim);
                System.out.println("System movement");

                try {
                    switch (e.getCode()) {
                        case W:
                            robot.move(Command.FORWARD, 1, exploredMap, RobotConstants.STEP_PER_SECOND);
                            robot.sense(exploredMap, map);
                            break;
                        case S:
                            robot.move(Command.BACKWARD, 1, exploredMap, RobotConstants.STEP_PER_SECOND);
                            robot.sense(exploredMap, map);
                            break;
                        case A:
                            robot.turn(Command.TURN_LEFT, RobotConstants.STEP_PER_SECOND);
                            robot.sense(exploredMap, map);
                            break;
                        case D:
                            robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
                            robot.sense(exploredMap, map);
                            break;
                        case I:
                            String to_send = String.format("I%d|%d|%s", 6, 3, Direction.getClockwise(Direction.UP).toString());
                            netMgr.send(to_send);
                            break;
                        default:
                            break;
                    }
                    System.out.println("Robot Direction AFTER:" + robot.getDir());
                } catch (InterruptedException ex) {
                    LOGGER.warning("Interrupt Exception");
                    ex.printStackTrace();
                }
            }
        });

        primaryStage.show();

        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent t) {
                Platform.exit();
                System.exit(0);
            }
        });

    } // end of start

    // Draw the Map Graphics Cells
    private void drawMap(boolean explored) {
        // Basic Init for the Cells
        gc.setStroke(MapConstants.CW_COLOR);
        gc.setLineWidth(5);
        // Draw the Cells on the Map Canvas
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                // Select Color of the Cells
                if (row <= MapConstants.STARTZONE_ROW + 1 && col <= MapConstants.STARTZONE_COL + 1)
                    gc.setFill(MapConstants.SZ_COLOR);
                else if (row >= MapConstants.GOALZONE_ROW - 1 && col >= MapConstants.GOALZONE_COL - 1)
                    gc.setFill(MapConstants.GZ_COLOR);
                else {

                    if (explored) {
                        if (exploredMap.getCell(row, col).isObstacle()) {
                            gc.setFill(MapConstants.OB_COLOR);
                        }
                        else if (exploredMap.getCell(row, col).isPath())
                            gc.setFill(MapConstants.PH_COLOR);
                        else if (exploredMap.getCell(row, col).isMoveThru())
                            gc.setFill(MapConstants.THRU_COLOR);
                        else if (exploredMap.getCell(row, col).isExplored())
                            gc.setFill(MapConstants.EX_COLOR);
                        else
                            gc.setFill(MapConstants.UE_COLOR);
                    } else {
                        if (exploredMap.getCell(row, col).isObstacle()) {
                            gc.setFill(MapConstants.OB_COLOR);
                        }
                        else {
                            gc.setFill(MapConstants.EX_COLOR);
                        }
                    }
                }

                // Draw the Cell on the Map based on the Position Indicated
                gc.strokeRect(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                                + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
                gc.fillRect(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                                + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
            }

            // Draw waypoint on the Map
            if (wayPoint != null) {
                gc.setFill(MapConstants.WP_COLOR);
                gc.fillRect(wayPoint.getX() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT
                                - wayPoint.getY() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
                gc.setFill(Color.BLACK);
                gc.fillText("W",
                        wayPoint.getX() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2
                                + MapConstants.CELL_CM / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT
                                - (wayPoint.getY() - 1) * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2
                                - MapConstants.CELL_CM / 2);
            }
        }

    }

    private void drawNewMap(boolean alreadyExplored) {
        // Basic Init for the Cells
        newGC.setStroke(MapConstants.CW_COLOR);
        newGC.setLineWidth(2);

        // Draw the Cells on the Map Canvas
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                // Select Color of the Cells
                if (row <= MapConstants.STARTZONE_ROW + 1 && col <= MapConstants.STARTZONE_COL + 1)
                    newGC.setFill(MapConstants.SZ_COLOR);
                else if (row >= MapConstants.GOALZONE_ROW - 1 && col >= MapConstants.GOALZONE_COL - 1)
                    newGC.setFill(MapConstants.GZ_COLOR);
                else {
                    if (alreadyExplored) {
                        if (newExploredMap.getCell(row, col).isObstacle()) {
                            newGC.setFill(MapConstants.OB_COLOR);
                        }
                        else if (newExploredMap.getCell(row, col).isPath())
                            newGC.setFill(MapConstants.PH_COLOR);
                        else if (newExploredMap.getCell(row, col).isMoveThru())
                            newGC.setFill(MapConstants.THRU_COLOR);
                        else if (newExploredMap.getCell(row, col).isExplored()) {
                            newGC.setFill(MapConstants.EX_COLOR);
                        }
                        else
                            newGC.setFill(MapConstants.UE_COLOR);
                    } else {

                        if (newExploredMap.getCell(row, col).isObstacle()) {
                            newGC.setFill(MapConstants.OB_COLOR);
                        }
                        else {
                            newGC.setFill(MapConstants.EX_COLOR);
                        }
                    }
                }

                // Draw the Cell on the Map based on the Position Indicated
                newGC.strokeRect(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                                + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
                newGC.fillRect(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                                + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
            }

            // Draw waypoint on the Map
            if (wayPoint != null) {
                newGC.setFill(MapConstants.WP_COLOR);
                newGC.fillRect(wayPoint.getX() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT
                                - wayPoint.getY() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                        MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);
                newGC.setFill(Color.BLACK);
                newGC.fillText("W",
                        wayPoint.getX() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2
                                + MapConstants.CELL_CM / 2,
                        (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT
                                - (wayPoint.getY() - 1) * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2
                                - MapConstants.CELL_CM / 2);
            }
        }
    }

    public static void main (String[] args) {
        launch(args);
    }

    // Mouse Event Handler for clicking and detecting Location
    private EventHandler<MouseEvent> MapClick = new EventHandler<MouseEvent>() {
        public void handle(MouseEvent event) {

            double mouseX = event.getX();
            double mouseY = event.getY();
            Boolean isMainMap = false;

            int selectedCol = (int) ((mouseX - MapConstants.MAP_OFFSET / 2) / MapConstants.MAP_CELL_SZ);
            int selectedRow = (int) (MapConstants.MAP_HEIGHT
                    - (mouseY - MapConstants.MAP_OFFSET / 2) / MapConstants.MAP_CELL_SZ);
            // Debug Text
            System.out.println(map.getCell(selectedRow, selectedCol).toString() + " validMove:"
                    + map.checkValidMove(selectedRow, selectedCol));

            if (setWaypoint) {
                System.out.println(setWayPoint(selectedRow, selectedCol)
                        ? "New WayPoint set at row: " + selectedRow + " col: " + selectedCol
                        : "Unable to put waypoint at obstacle or virtual wall!");
            }
            if (setRobot)
                System.out.println(setRobotLocation(selectedRow, selectedCol) ? "Robot Position has changed"
                        : "Unable to put Robot at obstacle or virtual wall!");
        }
    };

    private EventHandler<MouseEvent> NewMapClick = new EventHandler<MouseEvent>() {
        public void handle(MouseEvent event) {

            double mouseX = event.getX();
            double mouseY = event.getY();


            int selectedCol = (int) ((mouseX - MapConstants.MAP_OFFSET / 2) / MapConstants.MAP_CELL_SZ);
            int selectedRow = (int) (MapConstants.MAP_HEIGHT
                    - (mouseY - MapConstants.MAP_OFFSET / 2) / MapConstants.MAP_CELL_SZ);
            // Debug Text
            System.out.println(newExploredMap.getCell(selectedRow, selectedCol).toString() + " validMove:"
                    + newExploredMap.checkValidMove(selectedRow, selectedCol));

            if (event.getButton() == MouseButton.PRIMARY)
                System.out.println(setObstacle(newExploredMap, selectedRow, selectedCol)
                        ? "New Obstacle Added at row: " + selectedRow + " col: " + selectedCol
                        : "Obstacle at location alredy exists!");
            else
                System.out.println(removeObstacle(newExploredMap, selectedRow, selectedCol)
                        ? "Obstacle removed at row: " + selectedRow + " col: " + selectedCol
                        : "Obstacle at location does not exists!");

        }

    };

    // Place Obstacle at Location
    private boolean setObstacle(Map mapObj, int row, int col) {
        // Check to make sure the cell is valid and is not a existing obstacle
        if (mapObj.checkValidCell(row, col) && !mapObj.getCell(row, col).isObstacle()) {
            mapObj.getCell(row, col).setObstacle(true);

            // Set the virtual wall around the obstacle
            for (int r = row - 1; r <= row + 1; r++)
                for (int c = col - 1; c <= col + 1; c++)
                    if (mapObj.checkValidCell(r, c))
                        mapObj.getCell(r, c).setVirtualWall(true);

            return true;
        }
        return false;
    }

    // Remove Obstacle at Location
    private boolean removeObstacle(Map mapObj, int row, int col) {
        // Check to make sure the cell is valid and is not a existing obstacle
        if (mapObj.checkValidCell(row, col) && mapObj.getCell(row, col).isObstacle()) {
            mapObj.getCell(row, col).setObstacle(false);

            // Set the virtual wall around the obstacle
            for (int r = row - 1; r <= row + 1; r++)
                for (int c = col - 1; c <= col + 1; c++)
                    if (mapObj.checkValidCell(r, c))
                        mapObj.getCell(r, c).setVirtualWall(false);

            reinitVirtualWall();
            return true;
        }
        return false;
    }

    // Reinit virtual walls around obstacle
    private void reinitVirtualWall() {
        for (int row = 0; row < MapConstants.MAP_HEIGHT; row++) {
            for (int col = 0; col < MapConstants.MAP_WIDTH; col++) {
                if (map.getCell(row, col).isObstacle()) {
                    for (int r = row - 1; r <= row + 1; r++)
                        for (int c = col - 1; c <= col + 1; c++)
                            if (map.checkValidCell(r, c))
                                map.getCell(r, c).setVirtualWall(true);
                }
            }
        }
    }

    // Set waypoint
    private boolean setWayPoint(int row, int col) {
        if (exploredMap.wayPointClear(row, col)) {
            if (wayPoint != null)
                exploredMap.getCell(wayPoint).setWayPoint(false);

            wayPoint = new Point(col, row);
            wayPointTxt.setText(String.format("(%d, %d)", col, row));
            return true;
        } else
            return false;
    }

    // Set Robot Location and Rotate
    private boolean setRobotLocation(int row, int col) {
        if (map.checkValidMove(row, col)) {
            Point point = new Point(col, row);
            startPos.setLocation(col, row);
            startPosTxt.setText(String.format("(%d, %d)", col, row));
            robot.setStartPos(row, col, exploredMap);
            exploredMap.setAllExplored(true);
            System.out.println("Robot moved to new position at row: " + row + " col:" + col);
            return true;
        }
        return false;
    }

    private void init_before_exploration() {
        if (sim) {
            expMapDraw = true;
            robot.setSim(true);
            robot.setFindingFP(false);
            // reset to empty map
            exploredMap.resetMap();
            robot.setStartPos(startPos.y, startPos.x, exploredMap);
            robot.sense(exploredMap, map);
        }
        else {
            expMapDraw = true;
            robot.setSim(false);
            robot.setFindingFP(false);
            // reset to unexplored map
            exploredMap.resetMap();
            map = null;
            netMgr.initConn();
            String msg;
            wayPoint = null;
            startPos = null;
            // receive start exploration command from android
            if (!sim) {
                do {
                    msg = netMgr.receive();
                    LOGGER.info(msg);
                    //Set wayPoint
                    if (msg.contains(NetworkConstants.WAY_POINT_KEY)) {
                        LOGGER.info("Waypoint initialized.");
                        wayPoint = robot.parseWayPointJson(msg);
                        setWayPoint(wayPoint.y, wayPoint.x);
                    }
                    // set startPos 
//                    if (msg.contains(NetworkConstants.START_POINT_KEY)) {
//                        startPos = robot.parseStartPointJson(msg);
                     //change if start pos changes
                    robot.setStartPos(1, 1, exploredMap);
                } while(!msg.equals(NetworkConstants.START_EXP));
                                LOGGER.info("Receiving command to start exploration: " + msg);
                displayTimer.start();
                // initial sensing
                String calibrationCmd = robot.getCommand(Command.INITIAL_CALIBRATE, 1);
                netMgr.send(NetworkConstants.ARDUINO + calibrationCmd);
                do {
                    msg = netMgr.receive();
                    LOGGER.info(msg);
                } while(!msg.equals("F"));
                netMgr.send(NetworkConstants.ARDUINO + robot.getCommand(Command.SEND_SENSORS, RobotConstants.MOVE_STEPS));
                robot.sense(exploredMap, map);
            }
        }
    }

    // Event Handler for StartExpButton
    // not for actual run
    private EventHandler<MouseEvent> startBtnClick = new EventHandler<MouseEvent>() {
        @Override
        public void handle(MouseEvent event) {

            // a new task
            if (taskStarted == false && taskPaused == false) {
                startBtn.setText("Pause");
                displayTimer.stop();
                displayTimer.initialize();
                switch (taskSelected) {
                    case EXPLORATION:
                        init_before_exploration();  // refractor to be reused later in CASE image
                        // start thread
                        expTask = new Thread(new ExplorationTask());
                        startedTask = expTask;
                        taskStarted = true;
                        taskPaused = false;
                        expTask.start();
                        if(sim) {
                            displayTimer.start();
                        }
                        break;

                    case FASTEST_PATH:
                        if (sim) {
                            expMapDraw = true;
                            robot.setFindingFP(true);
                            exploredMap.removePaths();

                        }
                        else {
                            expMapDraw = true;
                            robot.setSim(false);
                            robot.setFindingFP(true);
                            exploredMap.removePaths();
                            netMgr.initConn();
                            robot.setStatus("Ready to start fastest path. Waiting for command.\n");
                        }
                        fastTask = new Thread(new FastTask());
                        startedTask = fastTask;
                        taskStarted = true;
                        taskPaused = false;
                        fastTask.start();
                        if (sim) {
                            displayTimer.start();
                        }

                        break;

                    case IMAGE:
                        init_before_exploration();
                        // start thread
                        imageTask = new Thread(new ImageTask());
                        startedTask = imageTask;
                        taskStarted = true;
                        taskPaused = false;
                        imageTask.start();
                        if(sim) {
                            displayTimer.start();
                        }
                        break;

                }
            }

            // pause a task
            else if (taskStarted == true && taskPaused == false) {
                startBtn.setText("Resume");
                startedTask.suspend();
                displayTimer.pause();
                taskStarted = false;
                taskPaused = true;
                //mapOutput.setText(mapDescriptor.generateMDFString(exploredMap));
            }

            // resume a task
            else if (taskStarted == false && taskPaused == true) {
                startBtn.setText("Pause");
                startedTask.resume();
                displayTimer.resume();
                taskStarted = true;
                taskPaused = false;
            }
        }
    };


    class ExplorationTask extends Task<Integer> {
        @Override
        protected Integer call() throws Exception {

            double coverageLimit = coverageLimitSB.getValue();
            int timeLimit = (int) (timeLimitSB.getValue() * 1000);
            int steps = (int) (stepsSB.getValue());

            Exploration explore = new Exploration(exploredMap, map, robot, coverageLimit, timeLimit, steps, sim);

            explore.exploration(new Point(MapConstants.STARTZONE_COL, MapConstants.STARTZONE_COL));
            System.out.println(Thread.currentThread().getName());

            robot.setStatus("Done exploration\n");

            if (!sim) {
                robot.sendAndroid(exploredMap);
            }
            displayTimer.stop();


            // Prepare for fastest path and wait for command from arduino
            System.out.println("Map Descriptor String 1: " + MapDescriptor.generateMDFString1(exploredMap));
            System.out.println("Map Descriptor String 2: " + MapDescriptor.generateMDFString2(exploredMap));

            if(!sim) {
                calibrate_and_start_fp();
            }



            return 1;
        }
    }

    class FastTask extends Task<Integer> {

        public String getFastTaskCmd(ArrayList<Command> commands) {
            System.out.println("getFastTaskCmd");
            Command tempCmd;
            int moves = 0;
            StringBuilder cmdBuilder = new StringBuilder();
            for (int i = 0; i < commands.size(); i++) {
                tempCmd = commands.get(i);
                if (tempCmd == Command.FORWARD && moves < RobotConstants.MAX_MOVE) {
                    moves++;

                    // if last cmd or moves == 9
                    if (i == commands.size() - 1 || moves == RobotConstants.MAX_MOVE) {
                        cmdBuilder.append(Command.ArduinoMove.values()[tempCmd.ordinal()]);
                        cmdBuilder.append(moves);
                        cmdBuilder.append('|');
                        moves = 0;
                    }
                }
                else {  // not forward
                    if (moves > 0) {    // previous forward, append the forward moves first
                        cmdBuilder.append(Command.ArduinoMove.values()[Command.FORWARD.ordinal()]);
                        cmdBuilder.append(moves);
                        cmdBuilder.append('|');

                    }

                    // turning
                    cmdBuilder.append(Command.ArduinoMove.values()[tempCmd.ordinal()]);
                    cmdBuilder.append('1');
                    cmdBuilder.append('|');


                    moves = 0;
                }
            }

            String cmds = cmdBuilder.toString();
            return cmds;
        }

        @Override
        protected Integer call() throws Exception {
            System.out.println("Calling fastest path task");

            // Calculate the path and make the first turn (if any) first during the 1min interval to save time
            double startT = System.currentTimeMillis();
            double endT = 0;
            FastestPath fp = new FastestPath(exploredMap, robot, sim);
            ArrayList<Cell> path;

            System.out.println("Running A-Star algorithm to find fastest path");
            System.out.println("Robot position x:" + robot.getPos().x + "y: " +robot.getPos().y + "waypoint: "+ wayPoint.x + "," + wayPoint.y);
            path = fp.runAStar(new Point(robot.getPos().x, robot.getPos().y), wayPoint, robot.getDir());
            System.out.println("Fastest path determined");
            path.addAll(fp.runAStar(wayPoint, new Point(MapConstants.GOALZONE_COL, MapConstants.GOALZONE_ROW), robot.getDir()));

            fp.displayFastestPath(path, true);

            ArrayList<Command> commands = fp.getPathCommands(path);

            // execute the first command if it is turning
            Command firstCmd = commands.get(0);
            if (firstCmd == Command.TURN_LEFT) {
                robot.turn(firstCmd, RobotConstants.STEP_PER_SECOND);
                //Remove executed command from commands ArrayList
                commands.remove(0);
            }

            // Get commands to execute fastest path in String format
            String cmd = getFastTaskCmd(commands);
            LOGGER.info("Checking FPCmdString: " + cmd);

            robot.setStatus("Ready to start fastest path. Waiting for command.\n");
            LOGGER.info(robot.getStatus());

            if(!sim) {
                System.out.println("Waiting for command...");
                // waiting for the fastest path command
                String msg;
                do {
                    msg = netMgr.receive();
                } while (!msg.equals(NetworkConstants.START_FP));

                LOGGER.info("Receiving command to start fastest path: " + msg);
                System.out.println(Thread.currentThread().getName());
                robot.setFindingFP(true);
                displayTimer.initialize();
                displayTimer.start();
            }

            String[] cmdStr = cmd.split("\\|");

            int steps = (int) stepsSB.getValue();
            char firstChar;
            int move;
            for (String c: cmdStr) {
                firstChar = c.charAt(0);
                System.out.println(firstChar);
                if (c.length() > 1) {
                    move = Integer.parseInt(c.substring(1));
                }
                else {
                    move = 1;
                }
                switch (firstChar) {
                    case 'W':
                        if (sim) {
                            robot.move(Command.FORWARD, move, exploredMap, steps);
                        }
                        else {
                            robot.move(Command.FORWARD, move, exploredMap, RobotConstants.STEP_PER_SECOND);
                            robot.sendAndroid(exploredMap);
                            netMgr.receive();
                        }
                        break;
                    case 'S':
                        if (sim) {
                            robot.move(Command.BACKWARD, move, exploredMap, steps);
                        }
                        else {
                            robot.move(Command.BACKWARD, move, exploredMap, RobotConstants.STEP_PER_SECOND);
                            robot.sendAndroid(exploredMap);
                            netMgr.receive();
                        }
                        break;
                    case 'D':
                        if (sim) {
                            robot.turn(Command.TURN_RIGHT, steps);
                        }
                        else {
                            // Added in for alignment
//                            robot.align_front_no_update();
//                            netMgr.receive();
                            robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
                            robot.sendAndroid(exploredMap);
                            // flush 3 sensor reading: align_front, turn, align_right
                            netMgr.receive();
                        }
                        break;
                    case 'A':
                        if (sim) {
                            robot.turn(Command.TURN_LEFT, steps);
                        }
                        else {
                            // Added in for alignment
//                            robot.align_front_no_update();
//                            netMgr.receive();
                            robot.turn(Command.TURN_LEFT, RobotConstants.STEP_PER_SECOND);
                            robot.sendAndroid(exploredMap);
                            // flush 3 sensor reading: align_front, turn, align_right
                            netMgr.receive();
                        }
                        break;
                }

            }
            endT = System.currentTimeMillis();
            int seconds = (int)((endT - startT)/1000%60);
            int minutes = (int)((endT - startT)/1000/60);
            displayTimer.stop();
            System.out.println("Total Time: "+minutes+"mins "+seconds+"seconds");
            return 1;
        }
    }

    //Algo with focus on image
    class ImageTask extends Task<Integer> {
        @Override
        protected Integer call() throws Exception {
//            Command c;

            double coverageLimit = coverageLimitSB.getValue();
            int timeLimit = (int) (timeLimitSB.getValue() * 1000);
            int steps = (int) (stepsSB.getValue());

            Exploration explore = new Exploration(exploredMap, map, robot, coverageLimit, timeLimit, steps, sim);
            explore.image_exploration(new Point(MapConstants.STARTZONE_COL, MapConstants.STARTZONE_COL));
            robot.setStatus("Done exploration\n");
            if (!sim) {
                robot.sendAndroid(exploredMap);
            }
            displayTimer.stop();

            // Prepare for fastest path and wait for command from arduino
            if(!sim) {

                calibrate_and_start_fp();

            }
            return 1;
        }
    }

    private void calibrate_and_start_fp() throws InterruptedException {
        // Calibration
        // pause for 5s, initially facing down, after calibration, should face up
        TimeUnit.MILLISECONDS.sleep(5000);
        while(robot.getDir()!= Direction.LEFT){
            robot.turn(Command.TURN_RIGHT, RobotConstants.STEP_PER_SECOND);
        }
        String calibrationCmd = robot.getCommand(Command.FINAL_CALIBRATE, 1);    // steps 1 for consistency
        netMgr.send(NetworkConstants.ARDUINO + calibrationCmd);
        robot.setDir(Direction.RIGHT);

        expMapDraw = true;
        robot.setFindingFP(true);
        exploredMap.removePaths();

        fastTask = new Thread(new FastTask());
        startedTask = fastTask;
        taskStarted = true;
        taskPaused = false;
        fastTask.start();
    }

    // Event Handler for resetMapBtn
    private EventHandler<MouseEvent> resetMapBtnClick = new EventHandler<MouseEvent>() {
        public void handle(MouseEvent event) {
            internalHandleResetMap();
        }
    };

    private void internalHandleResetMap() {

        // stop existing task and set startedTask to null if any
        // so that the startBtn is not set to not visible
        if (startedTask != null) {
            startedTask.stop();
            startedTask = null;
        }
        taskStarted = false;
        taskPaused = false;
        startBtn.setText("Start");
        displayTimer.stop();
        displayTimer.initialize();

        // reset the map and robot
        startBtn.setVisible(true);
        exploredMap.resetMap();
        mapDescriptor.loadRealMap(exploredMap, defaultMapPath);
        // just in case previously in actual mode map=null
        map = new Map();
        mapDescriptor.loadRealMap(map, defaultMapPath);
        startPos.setLocation(1, 1);
        startPosTxt.setText(String.format("(%d, %d)", 1, 1));
        if (wayPoint != null)
            exploredMap.getCell(wayPoint).setWayPoint(false);
        wayPoint.setLocation(MapConstants.GOALZONE_COL, MapConstants.GOALZONE_ROW);
        wayPointTxt.setText(String.format("(%d, %d)", MapConstants.GOALZONE_COL, MapConstants.GOALZONE_ROW));
        rightRB.setSelected(true);
        robot = new Robot(sim, false, 1, 1, Direction.RIGHT);
        robot.setStatus("Reset to Start Zone");

    }

    // Draw Method for Robot
    public void drawRobot() {
        gc.setStroke(RobotConstants.ROBOT_OUTLINE);
        gc.setLineWidth(2);

        gc.setFill(RobotConstants.ROBOT_BODY);

        int col = robot.getPos().x - 1;
        int row = robot.getPos().y + 1;
        int dirCol = 0, dirRow = 0;

        gc.strokeOval(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                        + MapConstants.MAP_OFFSET / 2,
                3 * MapConstants.MAP_CELL_SZ, 3 * MapConstants.MAP_CELL_SZ);
        gc.fillOval(col * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - row * MapConstants.MAP_CELL_SZ
                        + MapConstants.MAP_OFFSET / 2,
                3 * MapConstants.MAP_CELL_SZ, 3 * MapConstants.MAP_CELL_SZ);

        gc.setFill(RobotConstants.ROBOT_DIRECTION);
        switch (robot.getDir()) {
            case UP:
                dirCol = robot.getPos().x;
                dirRow = robot.getPos().y + 1;
                break;
            case DOWN:
                dirCol = robot.getPos().x;
                dirRow = robot.getPos().y - 1;
                break;
            case LEFT:
                dirCol = robot.getPos().x - 1;
                dirRow = robot.getPos().y;
                break;
            case RIGHT:
                dirCol = robot.getPos().x + 1;
                dirRow = robot.getPos().y;
                break;
        }
        gc.fillOval(dirCol * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                (MapConstants.MAP_CELL_SZ - 1) * MapConstants.MAP_HEIGHT - dirRow * MapConstants.MAP_CELL_SZ
                        + MapConstants.MAP_OFFSET / 2,
                MapConstants.MAP_CELL_SZ, MapConstants.MAP_CELL_SZ);

        gc.setFill(Color.BLACK);
        for (String sname : robot.getSensorList()) {
            Sensor s = robot.getSensorMap().get(sname);
            gc.fillText(s.getId(), s.getCol() * MapConstants.MAP_CELL_SZ + MapConstants.MAP_OFFSET / 2,
                    (MapConstants.MAP_CELL_SZ) * MapConstants.MAP_HEIGHT - s.getRow() * MapConstants.MAP_CELL_SZ
                            + MapConstants.MAP_OFFSET / 2);
        }

    }

}