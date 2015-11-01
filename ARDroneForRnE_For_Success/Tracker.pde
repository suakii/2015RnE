/**
 ARDrone Contest
 Tracker.pde
 Christian Nitschke
 */
import processing.video.*;
import jp.nyatla.nyar4psg.*;

//---------------------------------------------------------------------------------------------
//Public functions of class Tracker.pde
//  public float GetTargetWidth(int targetID) // printed width in [unit]
//  public boolean IsExistTarget(int targetID)
//  public int GetTargetCurrentMarkerID(int targetID) // ID of currently tracked nexted marker (no markers are searched inside detected marker)
//  public float GetTargetCurrentMarkerWidth(int targetID) // printed width in [unit]
//  public PVector[] GetTargetCurrentMarkerVertex2D(int targetID) // four corner vertices [pixels] of marker in image
//  public PMatrix3D GetTargetMatrix(int targetID) // target to camera transformation matrix
//  public PVector GetTargetPosition(int targetID)
//  public float GetTargetDistance(int targetID) // distance [unit] to origin
//  public float GetTargetPlaneDistance(int targetID) // distance [unit] to target plane 
//  public double GetTargetConfidence(int targetID)
//  public long GetTargetLife(int targetID)
//  public int GetTargetLostCount(int targetID)
//  public void DrawBackground(PImage img) // draw window background image
//  public void BeginTransform(PMatrix3D modelviewMatrix) // set rendering coordinate transformation from model to camera coordinates
//  public void EndTransform() // reset rendering coordinate transformation
public class Tracker {

  private MultiMarker nya;
  private Target[] t;

  private String path = "marker/";

  private PApplet parent;
  private int width;
  private int height;
  private String cparam_file;
  private NyAR4PsgConfig config;
  private int numTargets;
  private int numMarkersTarget;
  private float targetWidth;
  private float markerWidthRatio;
  private int numMarkers;
  private String unit;

  private double[] intrinsic_matrix;
  private double[] distortion_coeffs;

  private PrintWriter log;
  private int frameNum;
  private String[] logEvents = {
    "takeoff", "landing"
  };

  private int stackSize;


  //---------------------------------------------------------------------------------------------
  public Tracker(PApplet parent, int width, int height, int numTargets, int numMarkersTarget, float targetWidth, String unit) 
  {
    this.parent = parent;
    this.width = width;
    this.height = height;
    //cparam_file = "camera_para.dat";
    //this.cparam_file = cparam_file;
    //config = NyAR4PsgConfig.CONFIG_PSG;
    config = NyAR4PsgConfig.CONFIG_DEFAULT;
    this.numTargets = numTargets;
    this.numMarkersTarget = numMarkersTarget;
    this.targetWidth = targetWidth;
    //markerWidthRatio = 18.f/45.f; //0.4
    markerWidthRatio = 13.f/30.f; //0.4
    numMarkers = numTargets * numMarkersTarget;
    this.unit = unit;

    Init();
  }

  //---------------------------------------------------------------------------------------------
  private void Init() 
  {
    //println(MultiMarker.VERSION); 
    //nya = new MultiMarker(parent, width, height, cparam_file, config);

    Init_cparam(width, height);
    nya = new MultiMarker(parent, width, height, intrinsic_matrix, distortion_coeffs);
    nya.setARClipping(0.1, 100000);

    t = new Target[numTargets];

    stackSize = 0;

    // init log
    frameNum = -1;
    //log = createWriter("log/flight" + year() + nf(month(), 2) + nf(day(), 2) + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".log");
    //log = createWriter("flight.log");
    //String logHeader = "Time [ms], Frame number, Target ID, Target marker ID, Distance origin [" + unit + "], Distance plane [" + unit + "], Confidence, Life, Lost count";
    //for (int i=0; i<logEvents.length; i++) {
    //  logHeader += (", " + logEvents[i]);
    //}
    //log.println(logHeader);
    //log.println("Time [ms], Frame number, Target ID, Target marker ID, Distance origin [" + unit + "], Distance plane [" + unit + "], Confidence, Life, Lost count");
    //log.flush();

    // init targets
    for (int i=0; i<numTargets; i++) {

      int[] markerID = new int[numMarkersTarget];
      for (int j=0; j<numMarkersTarget; j++) {

        markerID[j] = i * numMarkersTarget + j;
        float markerWidth = targetWidth * pow(markerWidthRatio, (float)j);

        // from image
        String filepath = new String(path + String.format("%d", i+1) + String.format("%d", 2-numMarkersTarget + j+1) + ".png");
        PImage markerImg = loadImage(filepath);
        //println(filepath);
        int markerRes = 64;
        int markerEdgePerc = 15;
        nya.addARMarker(markerImg, markerRes, markerEdgePerc, markerWidth);

        //        // from *.patt
        //        String filepath = new String(path + prefixes[i] + String.format("%d", markerID[5-numMarkersTarget + j]) + ".patt");
        //        nya.addARMarker(filepath, markerWidth);
      }
      t[i] = new Target(targetWidth, markerWidthRatio, markerID);
    }
    return;
  }

  //---------------------------------------------------------------------------------------------
  public void Detect(PImage img) 
  {
    nya.detect(img);

    // detect markers and update target information
    for (int i=0; i<numMarkers; i++) {
      int tID = GetTargetID(i);
      int tmID = GetTargetMarkerID(i);

      if (!nya.isExistMarker(i)) {
        t[tID].markerExists[tmID] = false;
      }
      else {
        t[tID].markerExists[tmID] = true;
        t[tID].markerVertex2D[tmID] = nya.getMarkerVertex2D(i);
        t[tID].Mi2C[tmID] = nya.getMarkerMatrix(i);
        t[tID].markerConfidence[tmID] = nya.getConfidence(i);
        t[tID].markerLife[tmID] = nya.getLife(i);
        t[tID].markerLostCount[tmID] = nya.getLostCount(i);
      }
    }

    // process target information
    for (int i=0; i<numTargets; i++) {
      t[i].Refresh();
    }

    // log frame statistics
    //frameNum++;
    //LogFrame();

    return;
  }

  //---------------------------------------------------------------------------------------------
  private void Init_cparam(int width, int height) 
  {
    // rescale parameters, calibrated at (1280x720), to match rescaled camera image input 
    double[] s = new double[2];
    s[0] = (double)(width-1.0) / (1280.0-1.0);
    s[1] = (double)(height-1.0) / (720.0-1.0);

    // parameters calibrated from 14 Parrot AR.Drone 2.0 front cameras through Matlab 2014a (model equal to OpenCV)
    // intrinsic_matrix - This parameter 3x3 matrix is consistent with the value of the intrinsic_matrix cvCalibrateCamera2 OpenCV functions to output.
    intrinsic_matrix = new double[9]; // row-aligned
    intrinsic_matrix[0] = 949.217  * s[0]; // focal length x
    intrinsic_matrix[1] = 0.232762026239994 * s[0]; // skew
    intrinsic_matrix[2] = 486.535  * s[0]; // principal point x
    intrinsic_matrix[3] = 0.0;
    intrinsic_matrix[4] = 954.557  * s[1]; // focal length y
    intrinsic_matrix[5] = 269.170  * s[1]; // principal point y
    intrinsic_matrix[6] = 0.0;
    intrinsic_matrix[7] = 0.0;
    intrinsic_matrix[8] = 1.0;    

    // distortion_coeffs - This parameter 4x1 matrix is consistent with the value of the distortion_coeffs cvCalibrateCamera2 OpenCV functions to output.
    distortion_coeffs = new double[4];
    distortion_coeffs[0] = -0.609314; // radial 1
    distortion_coeffs[1] = 0.445474; // radial 2
    distortion_coeffs[2] = 0.001027; // tangential 1
    distortion_coeffs[3] = 0.005585; // tangential 2
    return;
  }

 

  //---------------------------------------------------------------------------------------------
  public float GetTargetWidth(int targetID)
  {
    return t[targetID].width;
  }

  //---------------------------------------------------------------------------------------------
  public boolean IsExistTarget(int targetID)
  {
    return t[targetID].exists;
  }

  //---------------------------------------------------------------------------------------------
  public int GetTargetCurrentMarkerID(int targetID)
  {
    return t[targetID].currentMarkerID;
  }  

  //---------------------------------------------------------------------------------------------
  public float GetTargetCurrentMarkerWidth(int targetID)
  {
    int ID = t[targetID].currentMarkerID;
    return t[targetID].markerWidth[ID];
  } 

  //---------------------------------------------------------------------------------------------
  public PVector[] GetTargetCurrentMarkerVertex2D(int targetID)
  {
    int ID = t[targetID].currentMarkerID;
    return t[targetID].markerVertex2D[ID];
  }

  //---------------------------------------------------------------------------------------------
  public PMatrix3D GetTargetMatrix(int targetID)
  {
    return new PMatrix3D(t[targetID].T2C);
  }

  //---------------------------------------------------------------------------------------------
  public PVector GetTargetPosition(int targetID)
  {    
    PMatrix3D m = t[targetID].T2C;
    return new PVector(m.m03, m.m13, m.m23);
  }

  //---------------------------------------------------------------------------------------------
  public float GetTargetDistance(int targetID)
  { 
    PMatrix3D m = t[targetID].T2C;
    float x = m.m03;
    float y = m.m13;
    float z = m.m23;
    return sqrt(x*x + y*y + z*z);
  }

  //---------------------------------------------------------------------------------------------
  public float GetTargetPlaneDistance(int targetID)
  { 
    PMatrix3D m = t[targetID].T2C;
    float z = m.m23;
    return abs(z);
  }

  //---------------------------------------------------------------------------------------------
  public double GetTargetConfidence(int targetID)
  {
    return t[targetID].confidence;
  }

  //---------------------------------------------------------------------------------------------
  public long GetTargetLife(int targetID)
  {
    return t[targetID].life;
  }

  //---------------------------------------------------------------------------------------------
  public int GetTargetLostCount(int targetID)
  {
    return t[targetID].lostCount;
  }

  //---------------------------------------------------------------------------------------------
  public void DrawBackground(PImage img) 
  {
    nya.drawBackground(img);
    return;
  }

  //---------------------------------------------------------------------------------------------
  public void BeginTransform(PMatrix3D modelviewMatrix) 
  {
    PGraphicsOpenGL pgl=((PGraphicsOpenGL)parent.g);
    //行列の待避
    pgl.pushProjection();
    nya.setARPerspective();

    //ModelViewの設定
    parent.pushMatrix();
    parent.setMatrix(modelviewMatrix);

    stackSize++;
    return;
  }

  //---------------------------------------------------------------------------------------------
  public void EndTransform() 
  {
    if (stackSize==0)
      return;

    //ModelViewの復帰
    parent.popMatrix();
    //Projectionの復帰
    PGraphicsOpenGL pgl=((PGraphicsOpenGL)parent.g);
    pgl.popProjection();

    stackSize--;
    return;
  }

  //---------------------------------------------------------------------------------------------
  private int GetTargetID(int markerID)
  {
    return markerID / numMarkersTarget;
  }

  //---------------------------------------------------------------------------------------------
  private int GetTargetMarkerID(int markerID)
  {
    return markerID % numMarkersTarget;
  }
}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
public class Target {

  // per target information
  private int numMarkers;
  private float width;
  private float markerWidthRatio;
  private boolean exists;
  private int currentMarkerID;
  private double confidence;
  private long life;
  private int lostCount; 
  private PMatrix3D T2C; // transformation Target i -> Camera
  private PMatrix3D C2T; // transformation Camera -> Target i

  // per marker information
  private int[] markerID;
  private float[] markerWidth;
  private boolean[] markerExists;
  private double[] markerConfidence;
  private long[] markerLife;
  private int[] markerLostCount; 
  private PVector[][] markerVertex2D;
  private PMatrix3D[] Mi2C; // transformation Marker i -> Camera

  public PMatrix3D I = new PMatrix3D(1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f);

  //---------------------------------------------------------------------------------------------
  public Target(float width, float markerWidthRatio, int[] markerID) 
  {
    this.width = width;
    this.markerWidthRatio = markerWidthRatio;
    this.markerID = markerID;
    numMarkers = markerID.length;

    Init();
  }

  //---------------------------------------------------------------------------------------------
  private void Init() {

    markerWidth = new float[numMarkers];
    markerExists = new boolean[numMarkers];
    markerVertex2D = new PVector[numMarkers][];
    Mi2C = new PMatrix3D[numMarkers];
    markerConfidence = new double[numMarkers];
    markerLife = new long[numMarkers];
    markerLostCount = new int[numMarkers];

    for (int i=0; i<numMarkers; i++) {
      markerWidth[i] = targetWidth * pow(markerWidthRatio, float(i));
      markerExists[i] = false;
    }

    return;
  }

  //---------------------------------------------------------------------------------------------
  // initializes the target compound structure
  public void Refresh() 
  {
    currentMarkerID = -1;
    for (int i=0; i<numMarkers; i++) {
      if (markerExists[i]) {
        currentMarkerID = i;
        break;
      }
    }

    if (currentMarkerID != -1) {
      exists = true;
      confidence = markerConfidence[currentMarkerID];
      life = markerLife[currentMarkerID];
      lostCount = markerLostCount[currentMarkerID];
      T2C = new PMatrix3D(Mi2C[currentMarkerID]);
      C2T = new PMatrix3D(T2C);
      C2T.invert();
    }
    else {
      exists = false;
    }

    return;
  }
}

