/* Pod class store all attributes and function regarding the pod of 
 cableBot which is the piece where all cable are connected
 */

class Pod {
  final float MIN_POD_HEIGHT_mm = 150;
  final float HFOV_DEG = 90;
  final float VFOV_DEG = 60;
  final float HFOV_RAD = HFOV_DEG/180 * PI;
  final float VFOV_RAD = VFOV_DEG/180 * PI;
  final float YAWLIMIT_LEFT = PI;
  final float YAWLIMIT_RIGHT = -PI;
  final float YAW_RANGE = YAWLIMIT_LEFT - YAWLIMIT_RIGHT;
  final int YAW_RES = 255;
  final float PITCH_LIMIT_UP = PI/2;
  final float PITCH_LIMIT_DOWN = -PI/2;
  final float PITCH_RANGE = PITCH_LIMIT_UP - PITCH_LIMIT_DOWN;
  final int PITCH_RES = 255;
  
  private int size = 80;
  private PVector presentCoordinates = new PVector(0, 0, WINCH_PROTO_HEIGHT_mm);
  private PVector goalCoordinates = new PVector(0, 0, WINCH_PROTO_HEIGHT_mm);
  private float distanceFromGround = WINCH_PROTO_HEIGHT_mm;
  private PVector gimbalHead = new PVector(size, 0, 0);
  private float gimbalYaw = 0;
  private float gimbalPitch = 0;
  private float gimbalRoll = 0;

  private float c = color(100,0);
  private float presentSpeed;
  private float goalSpeed = 10;

  private Button grabber = new Button(this.presentCoordinates.x, this.presentCoordinates.y, this.size);

  Pod() {
  }

  Pod(PVector coord) {
    super();
    this.presentCoordinates = coord.copy();
    this.goalCoordinates = this.presentCoordinates.copy();
    this.grabber.coordinates.set(this.goalCoordinates.x, this.goalCoordinates.y);
  }

  void setPresentCoordinates(PVector coord) {
    this.presentCoordinates = coord.copy();
  }

  PVector getPresentCoordinates() {
    return this.presentCoordinates.copy();
  }
  
  int getYawTic(){
    int result = int(map(this.gimbalYaw,YAWLIMIT_RIGHT, YAWLIMIT_LEFT, 0, YAW_RES));
    return result;
  }
  
  int getPitchTic(){
    int result = int(map(this.gimbalPitch,PITCH_LIMIT_DOWN, PITCH_LIMIT_UP, 0, PITCH_RES));
    return result;
  }

  void setGoalXY(PVector coord) {
    this.goalCoordinates.x = coord.x;
    this.goalCoordinates.y = coord.y;
    this.grabber.coordinates.set(coord.x, coord.y);
  }

  void setGoalZ(float z){
    this.goalCoordinates.z = z;
  }

  void offsetGoalZ(int offset) {
    this.goalCoordinates.z = max(this.goalCoordinates.z+offset, 0);
  }
  
  void offsetGimbal(float yaw, float pitch, float roll){
    this.gimbalYaw = max(min(this.gimbalYaw + yaw,this.YAWLIMIT_LEFT),this.YAWLIMIT_RIGHT);
    this.gimbalPitch = max(min(this.gimbalPitch + pitch,this.PITCH_LIMIT_UP),this.PITCH_LIMIT_DOWN);
    this.gimbalRoll += roll;
    gimbalHead.set(cos(this.gimbalYaw),sin(this.gimbalYaw), sin(this.gimbalPitch));
    gimbalHead.setMag(this.size);
  }

  PVector getGoalCoordinates() {
    return this.goalCoordinates.copy();
  }

  float getGoalSpeed() {
    return this.goalSpeed;
  }

  float getDistanceFromGround() {
    return this.distanceFromGround;
  }
  
  float getGimbalYaw(){
    return this.gimbalYaw;
  }

  void setDistanceFromGround(float value) {
    this.distanceFromGround = value;
  }

  void draw() {
    stroke(100,0,0);
    noFill();
    //draw presenst pod coordinates and gimbal
    pushMatrix();
    translate(this.presentCoordinates.x, this.presentCoordinates.y, this.presentCoordinates.z);
    ellipse(0, 0, this.size, this.size);
    rotateZ(this.gimbalYaw);
    rotateY(this.gimbalPitch);
    rotateX(this.gimbalRoll);
    translate(this.size/2,0,0);
    
    //draw camera
    fill(this.c);
    PVector[] camvertices = new PVector[4];
    camvertices[0] = new PVector(cos(HFOV_RAD/2),sin(HFOV_RAD/2),sin(VFOV_RAD/2));
    camvertices[0].setMag(this.size);
    camvertices[1] = new PVector(cos(HFOV_RAD/2),-sin(HFOV_RAD/2),sin(VFOV_RAD/2));
    camvertices[1].setMag(this.size);
    camvertices[2] = new PVector(cos(HFOV_RAD/2),-sin(HFOV_RAD/2),-sin(VFOV_RAD/2));
    camvertices[2].setMag(this.size);
    camvertices[3] = new PVector(cos(HFOV_RAD/2),sin(HFOV_RAD/2),-sin(VFOV_RAD/2));
    camvertices[3].setMag(this.size);
    for(int i=0;i<4;i++){
      int j = (i+1) % 4;
      beginShape(TRIANGLES);
      vertex(camvertices[i].x,camvertices[i].y,camvertices[i].z);
      vertex(camvertices[j].x,camvertices[j].y,camvertices[j].z);
      vertex(0,0,0);
      endShape(CLOSE);
    }
    box(this.size,this.size/3,this.size/3);
    popMatrix();  

    //draw goal pod coordinates
    stroke(0, 125, 150);
    noFill();
    translate(this.goalCoordinates.x, this.goalCoordinates.y, this.goalCoordinates.z);
    ellipse(0,0, this.size, this.size);
    translate(-this.goalCoordinates.x, -this.goalCoordinates.y, -this.goalCoordinates.z);

    //draw grabber
    this.grabber.drawButton();
  }
}

class Button {
  PVector coordinates;
  int size;  //button size
  color rectColor, rectHighlight;  //color states
  boolean grabbed = false;

  Button(float x, float y, int tmp_size) {
    this.coordinates = new PVector(x, y);
    this.size = tmp_size;
    this.rectColor = 100;
    this.rectHighlight = 150;
  }
  void setGrab(boolean isGrabbed) {
    this.grabbed = isGrabbed;
  }

  boolean getGrab() {
    return this.grabbed;
  }

  boolean isPointOverButton(PVector point) {
    if (point.dist(this.coordinates)<this.size) {  
      return true;
    } else {
      return false;
    }
  }

  void drawButton() {
    stroke(150);
    fill(50, 0, 150);
    ellipse(this.coordinates.x, this.coordinates.y, this.size, this.size);
  }
}
