/* //<>// //<>//
*  Calibrator class contains methods for gradient descent optimization of multiple cablebot winch length sampless
 */

class Calibrator {
  float[][] samples;
  int nbSamples = 25;
  float minSampleDistance;
  static final float MAX_GRADIENT_MAG = 50;
  static final float EPSILON = 0.01; //infinitesimal value in mm used in gradient computation 
  static final float L_RATE = 0.05; //learning rate of gradient descent
  CableBot mybot;

  Calibrator(CableBot abot) {
    mybot = abot;
    samples = new float[0][this.mybot.winchList.length];
  }

  void spreadWinchesCoord() {
    int n = this.mybot.winchList.length;
    PVector[] winchesCoord = new PVector[n];
    for (int i=0; i<n; i++) {
      PVector newcoord = new PVector();
      newcoord = PVector.fromAngle(-TWO_PI * i/n); //spread coordinates clockwise
      float Li = this.mybot.winchList[i].getPresentLength();
      newcoord.setMag(Li*cos(asin((WINCH_PROTO_HEIGHT_mm-mybot.pod.getDistanceFromGround())/Li)));
      newcoord.z=WINCH_PROTO_HEIGHT_mm;
      this.mybot.winchList[i].setCoordinates(newcoord);
      winchesCoord[i]=this.mybot.winchList[i].getCoordinates();
    }
    this.mybot.centerWinchesCoordinates(); //align and center winches along first 2 vetices (x axis convention)
  }

  void addSample() { //a sample is a set of winch length measurements and pod to ground measurement
    if (isNewSampleFarEnough()) {
      float[] newsample = new float[mybot.winchList.length+1];
      arrayCopy(this.mybot.getPresentLengths(), newsample);
      newsample[newsample.length-1] = this.mybot.pod.getDistanceFromGround();
      if (this.samples.length >= this.nbSamples) { //if samples array is full, delete last samples
        this.samples = (float[][]) reverse(this.samples);
        this.samples = (float[][]) shorten(this.samples);
        this.samples = (float[][]) reverse(this.samples);
      }
      this.samples = (float[][]) append(this.samples, newsample);
    }
  }

  boolean isNewSampleFarEnough() {
    boolean result;
    if (this.samples.length == 0) {
      result = true;
    } else {
      PVector lastPod = this.mybot.getPodFromLengths(this.samples[this.samples.length-1]);
      if (lastPod.dist(this.mybot.pod.getPresentCoordinates())>this.minSampleDistance) {
        result = true;
      } else {
        result = false;
      }
    }
    return result;
  }

  float costFunction() {
    float cost=0;
    PVector[] winchcoords = this.mybot.copyWinchCoords();
    for (float[] one_sample : this.samples) {
      float[] lengthMeasures = subset(one_sample, 0, one_sample.length-1);
      float distanceToGroundMeasure = one_sample[one_sample.length-1];
      PVector predictedPodCoords = trilateration(lengthMeasures, winchcoords);
      cost+= sq(predictedPodCoords.z - distanceToGroundMeasure);
    }
    cost/=2 * this.mybot.winchList.length;
    return cost;
  }

  PVector[] costGradient() {
    float cost = this.costFunction();
    PVector[] gradient = new PVector[this.mybot.winchList.length]; //for each winch, we use a 3d PVector for winch coordinates and, TODO a 2D PVector for zero offset and radius offset
    for (int i=0; i<this.mybot.winchList.length; i++) {
      float dfx, dfy, dfz;
      PVector offset = new PVector();

      offset.set(EPSILON, 0, 0);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfx = (this.costFunction()-cost)/EPSILON;

      offset.set(-EPSILON, EPSILON, 0);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfy = (this.costFunction()-cost)/EPSILON;

      offset.set(0, -EPSILON, EPSILON);
      this.mybot.winchList[i].offsetCoordinates(offset);
      dfz = 0;//(this.costFunction()-cost)/EPSILON; winch z coordinate is fixed, doesn't need calibration

      offset.set(0, 0, -EPSILON);
      this.mybot.winchList[i].offsetCoordinates(offset);
      gradient[i] = new PVector(dfx, dfy, dfz);
    }
    float gradMag = PVectorArrayMag(gradient);

    if (gradMag > MAX_GRADIENT_MAG) {
      for (PVector vect : gradient) {
        vect.mult(MAX_GRADIENT_MAG).div(gradMag);
      }
    }
    return gradient;
  }

  void optimize() {
    PVector[] gradient = this.costGradient();
    PVector[] winchesCoord = new PVector[gradient.length];
    for (int i=0; i<gradient.length; i++) {
      gradient[i].mult(-L_RATE);
      if (i==0) {
        gradient[i].x=0;
        gradient[i].y=0;
      }
      if (i==1)gradient[i].y=0;
      this.mybot.winchList[i].offsetCoordinates(gradient[i]);
      winchesCoord[i] = this.mybot.winchList[i].getCoordinates();
    }
  }

  void setMinSampleDistance(float distance) {
    this.minSampleDistance = distance;
  }

  void drawSamples() {
    strokeWeight(5);
    for (float[] s : this.samples) {
      PVector pod = new PVector();
      pod = this.mybot.getPodFromLengths(s).copy();
      stroke(0, 0, 255);
      point(pod.x, pod.y, pod.z);
      stroke(0, 60, 255);
      point(pod.x, pod.y, pod.z-s[s.length-1]);
    }
  }

  void drawCostValue() {
    pushMatrix();
    textAlign(LEFT);
    fill(30, 100, 100);
    textSize(TEXT_SIZE/2);
    camera();
    text("Calibration cost : " + this.costFunction(), width - 8 * TEXT_SIZE, TEXT_SIZE);
    popMatrix();
  }
}
