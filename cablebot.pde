/* 
 CableBot related class, with methods for x y and z pod set & read positions 
 */

class CableBot {

  final float GRID_SIZE = 5000; //GRID_SIZE 1px = 1mm
  final float GRID_RES = 100; //resoltion in between grid lines in mm

  Winch[] winchList;
  Pod pod;


  CableBot(int nbWinch) {
    this.pod = new Pod(new PVector(0.0, 0.0, WINCH_PROTO_HEIGHT_mm));
    this.winchList = new Winch[nbWinch];
    for (int i=0; i<nbWinch; i++) {
      this.winchList[i] = new Winch(i+1, 0); //winch id is 1+i (start at 1 on 430xl actuator
    }
  }

  /*
  *  Load & Save cablebot model, file used "data/cableBotModel.json"
   */

  void saveModel() {
    JSONArray model;
    model = new JSONArray();
    for (int i=0; i<this.winchList.length; i++) {
      JSONObject winchdata;
      PVector coordinates = this.winchList[i].getCoordinates();
      winchdata = new JSONObject();
      winchdata.setInt("id", i);
      winchdata.setString("x", str(coordinates.x));
      winchdata.setString("y", str(coordinates.y));
      winchdata.setString("z", str(coordinates.z));
      model.setJSONObject(i, winchdata);
    }
    JSONObject nb_winch = new JSONObject();
    nb_winch.setInt("nb_winch", this.winchList.length);
    model.setJSONObject(this.winchList.length, nb_winch);

    saveJSONArray(model, "data/cableBotModel.json");
    println("cableBot model saved");
  }

  void loadModel() {
    try {
      JSONArray model;
      model = loadJSONArray("data/cableBotModel.json");
      for (int i = 0; i < model.size(); i++) {
        JSONObject winchdata = model.getJSONObject(i);
        int id = winchdata.getInt("id");
        float x = float(winchdata.getString("x"));
        float y = float(winchdata.getString("y"));
        float z = float(winchdata.getString("z"));
        PVector coordinates = new PVector(x, y, z);
        this.winchList[i].setCoordinates(coordinates);
        this.winchList[i].setID(id);
      }
      println("cableBot model loaded");
    } 
    catch(Exception e) {
      println("cablebot model loading failed");
    }
  }

  /*
  *  update pod present coordinates from new winch lengths reading 
   *  &
   *  updates goal winch length from new pod goal coordinates
   */

  PVector[] getWinchesCoord() {
    PVector[] result = new PVector[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i] = this.winchList[i].getCoordinates();
    }
    return result;
  }

  float[] getPresentLengths() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i]=this.winchList[i].getPresentLength();
    }
    return result;
  }

  void setPresentPodFromWinchLengths() {
    PVector podPose = trilateration(this.getPresentLengths(), this.getWinchesCoord());
    this.pod.setPresentCoordinates(podPose);
  }

  //align and center a set of PVector according to the first edge along x axis
  void centerVectorArray(PVector[] vector_to_align) {

    PVector mean = new PVector(0, 0, 0);
    for (PVector vect : vector_to_align) {
      mean = mean.add(vect);
    }
    mean = mean.div(vector_to_align.length);
    mean.z=0;
    for (PVector vect : vector_to_align) {
      vect.sub(mean);
    }
    PVector Ex = new PVector(1, 0);
    PVector axis_to_align = vector_to_align[1].copy().sub(vector_to_align[0]);
    float angle = PVector.angleBetween(axis_to_align, Ex) - PI;
    for (PVector vect : vector_to_align) {
      vect.rotate(angle);
    }
  }

  void centerWinchesCoordinates() {
    this.centerVectorArray(this.getWinchesCoord());
  }

  PVector getPodFromLengths(float[] lengths) {
    PVector[] poses = copyWinchCoords();
    PVector result = trilateration(lengths, poses);
    return result;
  }

  //used only for simulation, use presentPod to compute
  void setPresentLengthFromPod() { 
    for (Winch w : this.winchList) {
      float Li = w.getCoordinates().dist(this.pod.getPresentCoordinates());
      w.setPresentLength(Li);
    }
  }

  //used only when no serial available
  void setPresentLengthFromLengths(Winch[] wincharray) { 
    for (int i=0; i<wincharray.length; i++) {
      this.winchList[i].setPresentLength(wincharray[i].getPresentLength());
    }
  }

  void setGoalLengthFromPod() {
    for (Winch w : this.winchList) {
      float Li = w.getCoordinates().dist(this.pod.getGoalCoordinates());
      w.setGoalLength(Li);
    }
  }

  void setGoalXY(PVector coord) {
    PVector goalInsideFootprint = polygonIntersection(new PVector(0, 0), coord, this.getWinchesCoord());
    this.pod.setGoalXY(goalInsideFootprint);
  }

  void setGoalXYZ(PVector coord) {
    this.setGoalXY(coord);
    float z = (coord.z<0)?0:coord.z;
    z = (coord.z>WINCH_PROTO_HEIGHT_mm)?WINCH_PROTO_HEIGHT_mm:z;
    this.pod.setGoalZ(z);
  }

  void translateGoalCoordinates(PVector shift) {
    this.setGoalXYZ(this.pod.getGoalCoordinates().add(shift));
  }

  float[] getGoalLengths() {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i]=this.winchList[i].getGoalLength();
    }
    return result;
  }

  float [] getLengthsFromPoint(PVector point) {
    float[] result = new float[this.winchList.length];
    for (int i=0; i<result.length; i++) {
      result[i]=point.dist(this.winchList[i].getCoordinates());
    }
    return result;
  }

  PVector[] copyWinchCoords() {
    PVector[] result = new PVector[this.winchList.length];
    for (int i=0; i<this.winchList.length; i++) {
      result[i] = new PVector();
      result[i]=this.winchList[i].getCoordinates().copy();
    }
    return result;
  }


  /*
  *  UI functions
   */

  void grabPod() {
    this.pod.grabber.setGrab(true);
  }

  void releaseGrab() {
    this.pod.grabber.setGrab(false);
  }

  boolean isGrabbed() {
    boolean result = this.pod.grabber.getGrab();
    return result;
  }

  boolean isPointOverGrabber(PVector point) {
    return this.pod.grabber.isPointOverButton(point);
  }


  /*
  *  Actuator driving functions
   */
  String setGoalLoad(float load) {
    String result = "";
    for (Winch w : this.winchList) {
      w.setGoalLoad(load);
      result += int(w.getGoalLoad())+ " ";
    }
    return result;
  }

  String setCompliantActuators() {
    String result = "";
    int loadoffset = 50;
    for (Winch w : this.winchList) {
      float newTorque;
      if (w.getGoalLoad()<w.getPresentLoad()) {
        newTorque = max(w.getGoalLoad()-loadoffset, 0);
      } else {
        newTorque = min(w.getGoalLoad()+loadoffset, MIN_WINCH_LOAD);
      }
      w.setGoalLoad(newTorque);
      result += int(w.getGoalLoad()) + " ";
    }
    return result;
  }

  String driveActuatorByLoad() {
    String result = "";
    float load = this.pod.getGoalCoordinates().z/WINCH_PROTO_HEIGHT_mm * MAX_WINCH_LOAD;

    for (Winch w : this.winchList) {
      PVector goalPodxy = this.pod.getGoalCoordinates().copy();
      goalPodxy.z = 0;
      PVector winchxy = w.getCoordinates().copy();
      winchxy.z = 0;
      w.setGoalLoad(min(load * winchxy.mag() / (2*goalPodxy.dist(winchxy)), MAX_WINCH_LOAD));
      result += int(w.getGoalLoad()) + " ";
    }
    return result;
  }

  String movePodToGoalPosition() {
    String result = "";
    float[] newLengths= this.getLengthsFromPoint(this.pod.getGoalCoordinates());
    for (int i = 0; i<this.winchList.length; i++) {
      this.winchList[i].setGoalLength(newLengths[i]);
      result += int((this.winchList[i].getGoalLength() + this.winchList[i].getZeroOffset() - MOUNT_LENGTH_in_mm)/POSITION_TO_FLOAT_mm) + " ";
    }
    return result;
  }

  String moveGimbal() {
    String result ="";
    result += this.pod.getYawTic() + " "; 
    result += this.pod.getPitchTic() + " ";
    return result;
  }


  /*
  *  drawing functions
   */

  PVector[] getUpDownLeftRightbounds(PVector cursor, PVector[] polygon) {
    PVector[] result=new PVector[4]; //4 vectors : 4 boundaries along +x +y -x -y
    int n = polygon.length;
    float minX = 0;
    float maxX = 0;
    float minY = 0;
    float maxY = 0;

    for (int i=0; i<n; i++) {
      maxX = (maxX < polygon[i].x) ? polygon[i].x : maxX;
      maxY = (maxY < polygon[i].y) ? polygon[i].y : maxY;
      minX = (minX > polygon[i].x) ? polygon[i].x : minX;
      minY = (minY > polygon[i].y) ? polygon[i].y : minY;
    }
    result[0] = polygonIntersection(cursor, new PVector(maxX, cursor.y), polygon);
    result[1] = polygonIntersection(cursor, new PVector(cursor.x, maxY), polygon);
    result[2] = polygonIntersection(cursor, new PVector(minX, cursor.y), polygon);
    result[3] = polygonIntersection(cursor, new PVector(cursor.x, minY), polygon);

    return result;
  }

  void drawGrid() {
    stroke(30);
    strokeWeight(1);
    for (int i=-(int)GRID_SIZE/2; i<(int)GRID_SIZE/2; i+=GRID_RES) {
      line(i, GRID_SIZE/2, i, -GRID_SIZE/2);
      line(GRID_SIZE/2, i, -GRID_SIZE/2, i);
    }
  }

  void drawCursorAxis() {
    strokeWeight(1);
    stroke(255, 0, 255);
    PVector[] podXYbound = this.getUpDownLeftRightbounds(this.pod.getGoalCoordinates(), this.copyWinchCoords());
    line(podXYbound[0].x, podXYbound[0].y, podXYbound[2].x, podXYbound[2].y);
    line(podXYbound[1].x, podXYbound[1].y, podXYbound[3].x, podXYbound[3].y);
  }

  void drawCables() {
    for (Winch w : this.winchList) {
      stroke(w.c, 100, 100);
      PVector winchcoord = w.getCoordinates();
      PVector podcoord = this.pod.getPresentCoordinates();
      line(winchcoord.x, winchcoord.y, winchcoord.z, podcoord.x, podcoord.y, podcoord.z);
    }
  }


  void drawWinches() {
    for (Winch w : this.winchList) {
      w.drawWinch();
    }
  }

  void drawPolygonOnGround(PVector[] poly) {
    int n = poly.length;
    for (int i=0; i<n; i++) {
      line(poly[i].x, poly[i].y, poly[(i+1)%n].x, poly[(i+1)%n].y);
    }
  }

  void drawFootprint() {
    this.drawPolygonOnGround(this.getWinchesCoord());
  }

  void drawBot() {   
    this.drawGrid();
    this.drawCables();
    this.pod.draw();
    this.drawWinches();
    this.drawFootprint();
    this.drawCursorAxis();
  }
}
