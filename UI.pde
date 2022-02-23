static class UI {
  static final float PODSHIFTMAG = 2;
  static float GIMBAL_SENSITIVITY = 0.02;
  static State state = State.COMPLIANT;
  static String stateName = "COMPLIANT";
  static int bufferSize = 50;
  static float[] XBuf = new float[bufferSize];
  static float[] YBuf = new float[bufferSize];
  static float[] ZBuf = new float[bufferSize];


  static boolean[] arrowKeyTable = {false, false, false, false, false, false};

  static void updateUserInputs(CableBot mybot, ControlDevice pad) {

    
    PVector podShift = new PVector(
      (arrowKeyTable[3]?1:0)*PODSHIFTMAG-(arrowKeyTable[2]?1:0)*PODSHIFTMAG, 
      (arrowKeyTable[0]?1:0)*PODSHIFTMAG -(arrowKeyTable[1]?1:0)*PODSHIFTMAG, 
      (arrowKeyTable[5]?1:0)*PODSHIFTMAG-(arrowKeyTable[4]?1:0)*PODSHIFTMAG);

    if (pad!=null) {
      float x = -(cos(mybot.pod.getGimbalYaw())*pad.getSlider("left_stick_y").getValue()-sin(mybot.pod.getGimbalYaw())*pad.getSlider("left_stick_x").getValue())*PODSHIFTMAG;
      float y = -(sin(mybot.pod.getGimbalYaw())*pad.getSlider("left_stick_y").getValue()+cos(mybot.pod.getGimbalYaw())*pad.getSlider("left_stick_x").getValue())*PODSHIFTMAG;
      float z = (pad.getSlider("RT").getValue()/2 - pad.getSlider("LT").getValue()/2)*PODSHIFTMAG;
      podShift.set(inputMean(XBuf, x), inputMean(YBuf, y), inputMean(ZBuf, z));
      float yawOffset = -pad.getSlider("right_stick_x").getValue()*GIMBAL_SENSITIVITY;
      float pitchOffset = pad.getSlider("right_stick_y").getValue()*GIMBAL_SENSITIVITY;
      mybot.pod.offsetGimbal(yawOffset, pitchOffset, 0);
    }

    mybot.translateGoalCoordinates(podShift);
  }

  static void updateBotOutput(CableBot mybot, Calibrator calibrator) {
    switch (UI.state) {
    case COMPLIANT :
      UI.stateName = "COMPLIANT";
      mybot.setPresentPodFromWinchLengths();
      calibrator.spreadWinchesCoord();
      break;
    case CALIBRATION :
      UI.stateName = "CALIBRATION";
      mybot.setPresentPodFromWinchLengths();
      calibrator.addSample();
      calibrator.optimize();
      calibrator.drawSamples();
      calibrator.drawCostValue();
      dataOut = mybot.movePodToGoalPosition();
      //dataOut += mybot.moveGimbal();
      break;
    case OPERATION :
      UI.stateName = "OPERATION";
      mybot.setPresentPodFromWinchLengths();
      dataOut = mybot.movePodToGoalPosition();
      break;
    }
    
  }

  static void mousePressedCallback(CameraControlManager mycam, CableBot mybot) {
    mycam.lastMouseClickedXY = mycam.mouseXY.copy();
  }

  static void mouseReleasedCallback(CameraControlManager mycam, CableBot mybot) {
    if (mybot.isGrabbed()) {
      mybot.pod.grabber.setGrab(false);
    } else {
      mycam.updateLastMouseReleased();
    }
  }

  static void mouseWheelCallback(CameraControlManager mycam, CableBot mybot, int wheelcount) {
    if (mybot.isPointOverGrabber(mycam.mouseOnGroundPlane)) {
      mybot.pod.offsetGoalZ(wheelcount);
    } else {
      mycam.orbitRadius += wheelcount;
    }
  }

  static void keyPressedCallback(CableBot mybot, Calibrator calibrator) {
    String mode_cmd = "P1 P2 P3";
    //String mode_cmd = "S1 S2 S3 S4";
    switch (UI.state) {
    case COMPLIANT :
      calibrator.setMinSampleDistance(SAMPLING_DISTANCE);
      mybot.setGoalXYZ(mybot.pod.getPresentCoordinates());
      if (myPort != null) myPort.write(mode_cmd+'\n');
      UI.state = State.CALIBRATION;
      break;
    case CALIBRATION :
      mybot.setGoalXYZ(mybot.pod.getPresentCoordinates());
      if (myPort != null) myPort.write(mode_cmd+'\n');
      mybot.centerWinchesCoordinates();
      UI.state = State.OPERATION;
      break;
    case OPERATION :
      break;
    }
  }

  static float inputMean(float[] buf, float input) {
    for (int i = buf.length-1; i>0; i--) {
      buf[i] = buf[i-1];
    }
    buf[0] = input;
    float mean = 0;
    for (int i=0; i<buf.length; i++) {
      mean += buf[i];
    }
    mean /= buf.length;
    return mean;
  }
}

void drawInfo(String str) {
  pushMatrix();
  textAlign(LEFT);
  fill(100, 100, 100);
  textSize(TEXT_SIZE/2);
  camera();
  text(str, 0, TEXT_SIZE);
  popMatrix();
}
