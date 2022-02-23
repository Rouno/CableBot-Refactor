//Compute pod 3d position from cableLengthData measurements and, takes only 3 points and 3 distances //<>//
//from https://en.wikipedia.org/wiki/Trilateration
//TODO : write code to manage 1 point 1 distance and 2 points 2 distances
PVector trilateration(float[] radius, PVector[] center) {
  float error=0;
  float r1 = radius[0];
  float r2 = radius[1];
  float r3 = radius[2];
  PVector P1, P2, P3, P1P2, P1P3 = new PVector();

  P1 = center[0].copy();
  P2 = center[1].copy();
  P3 = center[2].copy();
  P1P2 = P2.copy().sub(P1);
  P1P3 = P3.copy().sub(P1);

  //Compute unit vectors derived from first 3 first pillars coordinates
  PVector Ex = P1P2.copy().div(P1P2.mag());
  float i = Ex.copy().dot(P3.copy().sub(P1));
  PVector Ey = P1P3.copy().sub(Ex.copy().mult(i)).div(P1P3.copy().sub(Ex.copy().mult(i)).mag());
  PVector Ez = Ex.copy().cross(Ey);

  float d = P1P2.mag();
  float j = Ey.copy().dot(P3.copy().sub(P1));

  float x = (r1*r1 - r2*r2 + d*d)/(2*d);
  float y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - i/j*x;

  PVector result = new PVector();
  float z = 0;
  if (r1*r1 - x*x - y*y >=0 ) {
    z =  sqrt(r1*r1 - x*x - y*y);
    result = P1.add(Ex.mult(x)).add(Ey.mult(y)).add(Ez.mult(z)); //there may be an error on z.Ez sign !!!!!!!!!!!!!
  } else {
    //if 3 spheres intersection has no solution, grab closest solution following Al Kashi theoreme 
    //http://kuartin.math.pagesperso-orange.fr/theoremealkashi.htm
    //error from closest solution is stored in error
    error = sqrt(-r1*r1 + x*x + y*y);
    float cosalpha = (sq(d)-sq(r2)+sq(r1))/(2*r1*d);
    cosalpha = max(min(cosalpha, 1), -1); //ensure unique cosalpha solution by caping cosalpha between -1 and 1, appears when r1 & r2 cableLengthData cross each others
    float sinalpha = sqrt(1 - sq(cosalpha));
    result = P1.add(Ex.mult(cosalpha*r1).add(Ey.mult(sinalpha*r1)));
  }
  return result;
}


float PVectorArrayMag(PVector[] input) {
  float result = 0;
  for (PVector vect : input) {
    result+= sq(vect.mag());
  }
  result = sqrt(result);
  return result;
}

float sign (PVector p1, PVector p2, PVector p3)
{
  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

boolean PointInTriangle (PVector pt, PVector v1, PVector v2, PVector v3)
{
  boolean b1, b2, b3;

  b1 = sign(pt, v1, v2) < 0.0f;
  b2 = sign(pt, v2, v3) < 0.0f;
  b3 = sign(pt, v3, v1) < 0.0f;

  return ((b1 == b2) && (b2 == b3));
}

boolean PointInPolygon(PVector pt, PVector[] polygon) {
  boolean result = false;
  if (polygon.length<3) {
    println("polygon has less than 3 points");
    return result;
  }
  for (int i = 2; i<polygon.length; i++) {
    result |= PointInTriangle(pt, polygon[0], polygon[i-1], polygon[i]);
  }
  return result;
}


//implementing : https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//only case 3 considered
static PVector segmentsIntersection(PVector p1, PVector p2, PVector q1, PVector q2) {
  PVector result = p1.copy();
  PVector r = p2.copy().sub(p1);
  PVector s = q2.copy().sub(q1);
  PVector q_p = q1.copy().sub(p1);
  float rXs = r.cross(s).z;
  if (rXs != 0) {
    float t = q_p.cross(s).z/rXs;
    float u = q_p.cross(r).z/rXs;  
    boolean isIntersecting = (t>=0) && (t<=1) && (u>=0) && (u<=1);
    if (isIntersecting) { 
      result = p1.copy().add(r.mult(t));
    }
  }
  return result;
}

//computes p1p2 segment intersection with polygon, returns p2 if no intersection
static PVector polygonIntersection(PVector p1, PVector p2, PVector[] polygon) {
  int n = polygon.length;
  PVector result = p2.copy();
  for (int i = 0; i<n; i++) {
    PVector intersection = new PVector();
    intersection = segmentsIntersection(p1, p2, polygon[i], polygon[(i+1)%n]);
    if (! intersection.equals(p1)) {
      result = intersection;
    }
  }
  return result;
}
