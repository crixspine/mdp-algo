package Map;

import java.io.*;
import java.util.logging.*;

public class MapDescriptor {

    private static final Logger LOGGER = Logger.getLogger(MapDescriptor.class.getName());

    private String hexMapStr1;
    private String hexMapStr2;
    private String filename;

    /**
     * Construct Map descriptor for when there is no input real Map text file
     */
    public MapDescriptor() {
        hexMapStr1 = "";
        hexMapStr2 = "";
        filename = "";
    }

    /**
     * Construct Map descriptor with given real Map text file
     */
    public MapDescriptor(String filename) throws IOException {
        setHexMapStr(filename);
    }

    public String getHexMapStr1() {
        return hexMapStr1;
    }

    public String getHexMapStr2() {
        return hexMapStr2;
    }

    public String getFilename() {
        return filename;
    }

    public void setFilename(String filename) {
        this.filename = filename;
    }

    public void setHexMapStr(String filename) throws IOException {
        this.filename = filename;

        FileReader file = new FileReader(filename);
        BufferedReader buf = new BufferedReader(file);

        hexMapStr1 = buf.readLine();
        hexMapStr2 = buf.readLine();

        buf.close();
    }

    //Right pad "0" to the binary string so that its length is in multiple of 4
    private static String rightPadTo4(String biStr) {
        int check = biStr.length() % 4;
        if (check != 0) {
            int to_pad = 4 - check;
            LOGGER.log(Level.FINER, "Length of binary string not divisible by 4.");
            LOGGER.log(Level.FINER, "Length of string: {0}, Right Padding: {1}", new Object[]{biStr.length(), to_pad});
            StringBuilder padding = new StringBuilder();
            for (int i = 0; i < to_pad; i++) {
                padding.append('0');
            }
            biStr += padding.toString();
        }
        return biStr;
    }

    //Left pad "0" to binary string if it is not a multiple of 4, for conversion to hex string
    private String leftPadTo4(String biStr) {
        int check = biStr.length() % 4;
        if (check != 0) {
            int to_pad = 4 - check;
            LOGGER.log(Level.FINEST, "Length of binary string not divisible by 4.");
            LOGGER.log(Level.FINEST, "Length of string: {0}, Left Padding: {1}", new Object[]{biStr.length(), to_pad});
            StringBuilder padding = new StringBuilder();
            for (int i = 0; i < to_pad; i++) {
                padding.append('0');
            }
            biStr = padding.toString() + biStr;
        }
        return biStr;
    }

    //Convert binary str to hex str
    private static String biToHex(String biStr) {
        int dec = Integer.parseInt(biStr, 2);
        String hexStr = Integer.toHexString(dec);
        return hexStr;
    }

    //Convert hex str to binary str
    private String hexToBi(String hexStr) {
        String biStr = "";
        String tempBiStr = "";
        int tempDec;
        for (int i = 0; i < hexStr.length(); i++) {
            tempDec = Integer.parseInt(Character.toString(hexStr.charAt(i)), 16);
            tempBiStr = Integer.toBinaryString(tempDec);
            biStr += leftPadTo4(tempBiStr);
        }
        return biStr;
    }
    //part 1: unexplored cells '0', explored cells '1'
    public static String generateMDFString1(Map map) {
        StringBuilder MDFcreator1 = new StringBuilder();
        StringBuilder temp = new StringBuilder();
        temp.append("11"); //pad '11' in front
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                temp.append(map.getCell(r, c).isExplored() ? '1':'0');

                //convert to hex every 8 bits to avoid overflow
                if(temp.length() == 4) {
                    MDFcreator1.append(biToHex(temp.toString()));
                    temp.setLength(0);
                }
            }
        }
        temp.append("11"); //pad '11' at the end
        MDFcreator1.append(biToHex(temp.toString()));

        return MDFcreator1.toString();
    }

    //part 2: for explored cells: not obstacle '0', obstacle '0'
    public static String generateMDFString2(Map map) {
        StringBuilder MDFcreator2 = new StringBuilder();
        StringBuilder temp = new StringBuilder();
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                if (map.getCell(r, c).isExplored()) {
                    temp.append(map.getCell(r, c).isObstacle() ? '1' : '0');
                    if (temp.length() == 4) {
                        MDFcreator2.append(biToHex(temp.toString()));
                        temp.setLength(0);
                    }
                }
            }
        }

        //Right pad to 4 for last byte
        if(temp.length() % 4 != 0) {
            String tempBiStr = rightPadTo4(temp.toString());
            MDFcreator2.append(biToHex(tempBiStr));
        }

        return MDFcreator2.toString();

    }

 /**   public static String generateMDFString(Map map) {
        StringBuilder MDFcreator = new StringBuilder();
        StringBuilder temp = new StringBuilder();
        StringBuilder temp2 = new StringBuilder();
        StringBuilder temp2Hex = new StringBuilder();
        temp.append("11");
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                temp.append(map.getCell(r, c).isExplored() ? '1':'0');
                //convert to hex every 8 bits to avoid overflow
                if(map.getCell(r,c).isExplored()){
                    if(map.getCell(r,c).isObstacle()){
                        temp2.append('1');
                    }
                    else{
                        temp2.append('0');
                    }
                }
                if(temp.length() == 4) {
                    MDFcreator.append(biToHex(temp.toString()));
                    temp.setLength(0);
                }
                if (temp2.length() == 4){
                    temp2Hex.append(biToHex(temp2.toString()));
                    temp2.setLength(0);
                }
            }
        }
        // last byte
        temp.append("11");
        MDFcreator.append('\n');
        MDFcreator.append(temp2Hex);
        MDFcreator.append(biToHex(temp2.toString()));

        return MDFcreator.toString();
    }
**/
    //load explored arena in map
    private void loadMDFString1(String MDFstr1, Map map) {
        String expStr = hexToBi(MDFstr1);
        int index = 2;
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                if (expStr.charAt(index) == '1') {
                    map.getCell(r, c).setExplored(true);
                }
                index++;
            }
        }
    }

    public void loadMDFString2(String MDFstr2, Map map) {
        String obsStr = hexToBi(MDFstr2);
        int index = 0;
        for (int r = 0; r < MapConstants.MAP_HEIGHT; r++) {
            for (int c = 0; c < MapConstants.MAP_WIDTH; c++) {
                Cell cell = map.getCell(r, c);
                if (cell.isExplored()) {
                    if (obsStr.charAt(index) == '1') {
                        cell.setObstacle(true);
                        // create virtual wall
                        map.setVirtualWall(cell, true);
                    }
                    index++;
                }
            }
        }
    }

    //load real map into simulator
    public void loadRealMap(Map map, String filename) {
        this.filename = filename;
        try {
            setHexMapStr(filename);
        } catch (IOException e) {
            LOGGER.warning("IOException");
            e.printStackTrace();
        }
        loadMDFString1(this.hexMapStr1, map);
        loadMDFString2(this.hexMapStr2, map);
    }

    //save map into txt file
    public void saveRealMap(Map map, String filename) {
        try {

            FileWriter file = new FileWriter(filename);

            BufferedWriter buf = new BufferedWriter(file);
            String mapDes = generateMDFString1(map);
            buf.write(mapDes);
            buf.newLine();

            mapDes = generateMDFString2(map);
            buf.write(mapDes);
            buf.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
