import processing.serial.*;

static Serial myPort;      // The serial port
static final int NEW_LINE = '\n';
static final int NB_DATA_PER_ACTUATOR = 3;
static boolean firstContact = true;
static String dataOut = "";
static String[] dataIn;

static void serialCallBack(CableBot abot, Serial aport) {
  if (firstContact || abot==null) {
    aport.clear();
    firstContact = false;
    return;
  }
  String rawdataIn = aport.readString();
  rawdataIn = rawdataIn.substring(0,rawdataIn.length()-1); //erase NEW LINE character at the end of the string

  if (rawdataIn.length()>1) {
    dataIn = splitTokens(rawdataIn, " ");
    receiveFrame(abot, dataIn);
    aport.clear();
    myPort.write(dataOut+'\n');
    dataOut = "";
  }
}

static void receiveFrame(CableBot abot, String[] tokens) {
  for (int i = 0; i<abot.winchList.length; i++) {
    float len = float(tokens[i*NB_DATA_PER_ACTUATOR]) * POSITION_TO_FLOAT_mm - abot.winchList[i].getZeroOffset() + MOUNT_LENGTH_in_mm;
    float speed = float(tokens[i*NB_DATA_PER_ACTUATOR+1]) * SPEED_TO_FLOAT_mms;
    float load = float(tokens[i*NB_DATA_PER_ACTUATOR+2]);
    abot.winchList[i].setPresentValues(len, speed, load);
  }

  if (tokens[NB_DATA_PER_ACTUATOR*NB_ACTUATORS].length() > 1) {
    float USmeasure = float(tokens[NB_DATA_PER_ACTUATOR*NB_ACTUATORS]) + US_OFFSET_mm;
    abot.pod.setDistanceFromGround(USmeasure);
  }
} 
