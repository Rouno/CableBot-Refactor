import org.gamecontrolplus.gui.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;

ControlIO control;
Configuration config;
ControlDevice gpad;

CameraControlManager cameraControls;
CableBot myCableBot; //used to simulate bot when serial not available
CableBot simulatedBot;
Calibrator myCalibrator;

static final int NB_ACTUATORS = 3;
static final float POSITION_TO_FLOAT_mm = 0.0379;//845/22307;//0.075;
static final float SPEED_TO_FLOAT_mms = 0.479;
static final float WINCH_PROTO_HEIGHT_mm =510; //483;
static final float MOUNT_LENGTH_in_mm = 80;
static final float US_OFFSET_mm = 50;
static final float MAX_WINCH_LOAD = 1000;
static final float MIN_WINCH_LOAD = 200;
static final float MAX_WINCH_SPEED = 30;
static final float SAMPLING_DISTANCE = 150;

static int TEXT_SIZE; //size of text used

enum State {
  COMPLIANT, CALIBRATION, OPERATION
};

void setup() {
  //setup() should always start with serial init to determine if we need to simulate cablebot or not
  try {
    myPort = new Serial(this, "/dev/tty.usbmodem14101", 57600);
    myPort.bufferUntil(NEW_LINE);
    myCableBot = new CableBot(NB_ACTUATORS);
  }
  catch (Exception e) {
    println("Serial port initialization failed, running simulation instead");
    simulatedBot = new CableBot(NB_ACTUATORS);
    simulatedBot.loadModel();
    simulatedBot.centerWinchesCoordinates();
    myCableBot = new CableBot(simulatedBot.winchList.length);
    myCableBot.setPresentLengthFromLengths(simulatedBot.winchList);
  }

  myCalibrator = new Calibrator(myCableBot);

  size(1280, 720, P3D);
  TEXT_SIZE = height/20;
  rectMode(CENTER);
  colorMode(HSB, 100);
  cameraControls = new CameraControlManager((PGraphicsOpenGL) this.g);

  // Initialize the ControlIO for gamecontroller support
  control = ControlIO.getInstance(this);
  // Find a device that matches the configuration file
  gpad = control.getMatchedDeviceSilent("xbox_controller");
  if (gpad == null) {
    println("No suitable device configured");
  }
}

void draw() {
  background(0);
  cameraControls.updateMouse(mouseX,mouseY);
  if (mousePressed) {
    cameraControls.updateOrbitAngle();
  }
  cameraControls.updateCamera();
  
  UI.updateUserInputs(myCableBot, gpad);
  UI.updateBotOutput(myCableBot, myCalibrator);
  
  scale(1, -1); //invert y axis orientation to get correctly oriented x y z axis
  myCableBot.drawBot();
  drawInfo(UI.stateName);

  if (simulatedBot!=null) {
    simulatedBot.pod.setPresentCoordinates(myCableBot.pod.getGoalCoordinates());
    simulatedBot.setPresentLengthFromPod();
    myCableBot.pod.setDistanceFromGround(simulatedBot.pod.getPresentCoordinates().z);
    myCableBot.setPresentLengthFromLengths(simulatedBot.winchList);
  }
}

void serialEvent(Serial myPort) {
  serialCallBack(myCableBot, myPort);
}

void mousePressed() {
  UI.mousePressedCallback(cameraControls, myCableBot);
}

void mouseReleased() {
  UI.mouseReleasedCallback(cameraControls, myCableBot);
}

void mouseWheel(MouseEvent event) {
  UI.mouseWheelCallback(cameraControls, myCableBot, event.getCount());
}

void keyPressed() {
  switch (key) {
  case 's' :
    myCableBot.saveModel();
    break;
  case 'l' : 
    myCableBot.loadModel();
    break;
  case CODED :
    switch (keyCode) {
    case UP:
      UI.arrowKeyTable[0] = true;
      break;
    case DOWN:
      UI.arrowKeyTable[1] = true;
      break;
    case LEFT:
      UI.arrowKeyTable[2] = true;
      break;
    case RIGHT:
      UI.arrowKeyTable[3] = true;
      break;
    case CONTROL:
      UI.arrowKeyTable[4] = true;
      break;
    case SHIFT:
      UI.arrowKeyTable[5] = true;
      break;
    }
    break;
  default :
    UI.keyPressedCallback(myCableBot, myCalibrator);
  }
}

void keyReleased() {
  if (key== CODED) {
    switch (keyCode) {
    case UP:
      UI.arrowKeyTable[0] = false;
      break;
    case DOWN:
      UI.arrowKeyTable[1] = false;
      break;
    case LEFT:
      UI.arrowKeyTable[2] = false;
      break;
    case RIGHT:
      UI.arrowKeyTable[3] = false;
      break;
    case CONTROL:
      UI.arrowKeyTable[4] = false;
      break;
    case SHIFT:
      UI.arrowKeyTable[5] = false;
      break;
    }
  }
}
