
import com.shigeodayo.ardrone.manager.*;
import com.shigeodayo.ardrone.navdata.*;
import com.shigeodayo.ardrone.utils.*;
import com.shigeodayo.ardrone.processing.*;
import com.shigeodayo.ardrone.command.*;
import com.shigeodayo.ardrone.*;
import com.shigeodayo.ardrone.video.*;
import processing.video.*;
import jp.nyatla.nyar4psg.*;
import javax.media.opengl.*;

import com.hamoid.*;
VideoExport videoExport;
ARDroneForP5 ardrone;
MultiMarker nya;

boolean showTargetBall = true;//show center ball image
boolean trackingStart = false;
boolean isFlying = false;
float speedX = 0.0;
float speedY = 0.0;
float speedZ = 0.0;

float altitude =0.0;

float speedXKp = 0.1;
float speedXKi = 0.005;
float speedXKd = 0.01; 

float speedYKp = 0.1;
float speedYKi = 0.005;
float speedYKd = 0.01;




PIDController pidspeedX;// = new PIDController(speedXKp, speedXKi, speedXKd, 0.10);
PIDController pidspeedY;// = new PIDController(speedYKp, speedYKi, speedYKd, 0.10);

float speedZKp = 0.001;
float speedZKi = 0.000001;
float speedZKd = 0.001;


PIDController pidspeedZ;// = new PIDController(speedZKp, speedZKi, speedZKd, 0.10);

LowPassFilter altitudeLPF= new LowPassFilter(0.9);


//Log
PrintWriter log;


Tracker tracker;
int numTargets = 1;
int numMarkersTarget = 1;
//float targetWidth = 14.0f;
float targetWidth = 23.5;
String unit = "cm";

int targetId = 0;


float dt = .01;


void setup() {
  
 
 pidspeedX = new PIDController(speedXKp, speedXKi, speedXKd, 10);
 pidspeedY = new PIDController(speedYKp, speedYKi, speedYKd, 10);
 pidspeedZ = new PIDController(speedZKp, speedZKi, speedZKd, 10);
  
  
  
  
  
  size(640, 360, P3D);
  
  colorMode(RGB, 100);
  tracker = new Tracker(this, width, height, numTargets, numMarkersTarget, targetWidth, unit);
  
  
  log = createWriter("log/flight" + year() + nf(month(), 2) + nf(day(), 2) + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".log");
  videoExport = new VideoExport(this, ""+ year() + nf(month(), 2) + nf(day(), 2) + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".mp4");
  
  log.println("x" + "\t" + "y" + "\t" + "z" + "\t" + "distance" + "\t"+ "speedX" + "\t" + "speedY" +"\t" + "P_INPUT_X" + "\t" + "P_INPUT_Y");

  //log.flush();
  
  ardrone=new ARDroneForP5("192.168.1.1");
  ardrone.connect();
  ardrone.connectNav();
  ardrone.connectVideo();
  ardrone.start();
  
  
}


int count = 0;

float GAIN = 0.4;
float thA = 10.0;
float REF_altitude = 1500.0;
boolean isRefAltitude = false;

void draw() {
  background(204);
  PImage img=ardrone.getVideoImage(true);
  if (img==null)
    return;

  
   if(isFlying == true) {
   float paltitude = ardrone.getAltitude();
   float P_INPUT = abs(GAIN * (REF_altitude - paltitude));
   if (P_INPUT > 50) {
     P_INPUT = 50;
   }
   if(paltitude < REF_altitude - thA){
    if(!isRefAltitude){
      ardrone.up((int)P_INPUT); // propotional control
    }
  }
  else if(paltitude > REF_altitude + thA){
    if(!isRefAltitude){
      ardrone.down((int)P_INPUT); // propotional control
    }
  }
  else{
      isRefAltitude = true;
      ardrone.stop();
  }
  
  
  
  
}
   

  hint(DISABLE_DEPTH_TEST);
  image(img, 0, 0);
  hint(ENABLE_DEPTH_TEST);

  int battery = ardrone.getBatteryPercentage();

  drawBattery(battery);

  
  tracker.Detect(img);
  background(0);
  tracker.DrawBackground(img);
  
   for (int i=0; i<numTargets; i++) {
    if (!tracker.IsExistTarget(i)) {
      if (trackingStart ==true && isFlying==true) {
        pidspeedX.reSet();
        pidspeedY.reSet();
        altitudeLPF.reSet();
        
        //target miss;
        ardrone.stop();
      }
      return;
      }
      // draw marker contour
      PVector[] v = tracker.GetTargetCurrentMarkerVertex2D(i);  
      colorMode(HSB, 360, 100, 100, 100);
      stroke(100, 100, 100);
      strokeWeight(4);
      noFill();
      beginShape();
      vertex(v[0].x, v[0].y);      
      vertex(v[1].x, v[1].y);
      vertex(v[2].x, v[2].y);
      vertex(v[3].x, v[3].y);
      endShape(CLOSE);
      
  }
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
    //draw status bar - need to resize image size;
    //^^
       
  // get marker position
    PVector P = tracker.GetTargetPosition(targetId);
    float x = P.x;// *  10; //[cm -> mm]
    float y = P.y;// *  10; //[cm -> mm]
    float z = P.z;// * -10; //[cm -> mm]
    
    PVector markerLoc = new PVector(x, y);
    PVector centerLoc = new PVector(width/2, height/2);
    PVector v = PVector.sub(markerLoc, centerLoc);
    v.normalize();
    
    if ((int)speedX!=0 || (int)speedY!=0) {
      drawVector(v, centerLoc);
    }
  
    float distance = tracker.GetTargetDistance(targetId);//height for our projects;
    altitude = altitudeLPF.estimate(ardrone.getAltitude());
    
  
    //---PIDs
    if(trackingStart == true && isFlying == true) {
      for(int j = 0; j < 50; j++) {
        speedY += ((int)(pidspeedY.estimate((int)y, 0))*0.01);
        speedX += ((int)(pidspeedX.estimate((int)x, 0))*0.01);
        //speed x acquire and apply instant....
        //true moving code is needed....
        if (y > 0) {
            ardrone.backward(speedY);
            text("backward", width/128*50, height/20);
            return;
          }
          else if (y < 0) {
            ardrone.forward(speedY);
            text("forward", width/128*50, height/20);
            return;
          }
    
          if (x > 0) {
            ardrone.goRight(speedX);
            text("goRight", width/128*50, height/20);
            return;
          } 
          else if (x < 0 ) {
            ardrone.goLeft(speedX);
            text("goLeft", width/128*50, height/20);
            return;
          }
          P = tracker.GetTargetPosition(targetId);
          x = P.x;
          y = P.y;
          z = P.z;
          log.println(x + "\t" + y + "\t" + z + "\t" + distance +"\t"+ speedX + "\t" + speedY );
  
      }
  }
    pidspeedX.reSet();
    pidspeedY.reSet();

    

    /*
    //speedX= pidspeedX.estimate(y, 0);
    //speedY= pidspeedY.estimate(x, 0);
    //speedZ= -(pidspeedZ.estimate(altitude - 1000, 0));
    
    
    text(x + "," + y + "," + z, width/128*50, height/12);
    float dist = 1500;
    float thx = 10;
    float thy = 10;
    float thz = 10;
    //trackingStart = isFlying = true;
    //not using pid controller. - suakii
    if (trackingStart == true && isFlying==true) { 
       
         float P_INPUT_Y = abs(GAIN * (0 - y));
         if (P_INPUT_Y > 50) {
           P_INPUT_Y = 50;
         }
         float P_INPUT_X = abs(GAIN * (0 - x));
         if (P_INPUT_X > 50) {
           P_INPUT_X = 50;
         }
         
       log.println(x + "\t" + y + "\t" + z + "\t" + distance +"\t"+ speedX + "\t" + speedY +"\t" + P_INPUT_X + "\t" + P_INPUT_Y);
      
       if (y > 0 + thy) {
          ardrone.backward((int)P_INPUT_Y);
          text("backward", width/128*50, height/20);
          return;
        }
        else if (y < 0 - thy) {
          ardrone.forward((int)P_INPUT_Y);
          text("forward", width/128*50, height/20);
          return;
        }
  
        if (x > 0 + thx) {
          ardrone.goRight((int)P_INPUT_X);
          text("goRight", width/128*50, height/20);
          return;
        } 
        else if (x < 0 - thx) {
          ardrone.goLeft((int)P_INPUT_X);
          text("goLeft", width/128*50, height/20);
          return;
        }
        
        if ((z - dist) > thz) {
          //ardrone.down();
          text("down", width/128*50, height/20);
          return;
        } 
        else if ((z - dist) < -thz) {
          //ardrone.up();
          text("up", width/128*50, height/20);
          return;
        }
        
        ardrone.stop(); 
        text("stop", width/128*50, height/20);        
    }
    */
    
  
  
  videoExport.saveFrame();

  
}


void keyPressed() {
  if (key==CODED) {
    if (keyCode==UP) {
      if (isFlying==true) {
        ardrone.move3D(10, 0, 0, 0);//forward
      }
    }
    else if (keyCode==DOWN) {
      if (isFlying==true) {
        ardrone.move3D(-10, 0, 0, 0);//backward
      }
    }
    else if (keyCode==LEFT) {
      if (isFlying==true) {
        ardrone.move3D(0, 10, 0, 0);//go left
      }
    }
    else if (keyCode==RIGHT) {
      if (isFlying==true) {
        ardrone.move3D(0, -10, 0, 0);//go right
      }
    }
    else if (keyCode==SHIFT) {
      isFlying=true;
      ardrone.takeOff();//take off
    }
    else if (keyCode==CONTROL) {
      isFlying=false;
      trackingStart = false;
      ardrone.landing();//land
      log.flush();
      log.close();
    }
  }
  else {
    if (key=='s') {
      ardrone.stop();
    }
    else if (key=='r') {
      if (isFlying==true) {
        ardrone.spinRight();
      }
    }
    else if (key=='l') {
      if (isFlying==true) {
        ardrone.spinLeft();
      }
    }
    else if (key=='u') {
      if (isFlying==true) {
        ardrone.up();
      }
    }
    else if (key=='d') {
      if (isFlying==true) {
        ardrone.down();
      }
    }
    else if (key=='1') {
      ardrone.setHorizontalCamera();
    }
    else if (key=='3') {
      ardrone.setVerticalCamera();
    }
    else if (key=='5') {
      ardrone.toggleCamera();
    }
    else if (key == '6') {
      trackingStart = true;
    }
  }
}




void drawVector(PVector v, PVector loc) {
  pushMatrix();
  translate(loc.x, loc.y);
  stroke(0, 255, 0);
  strokeWeight(1);
  rotate(v.heading2D()); 
  line(1, 0, -4, +5);
  line(1, 0, -4, -5);
  popMatrix();
}



void drawBattery(int battery) {
  stroke(255, 255, 255);
  strokeWeight(2.1);
  noFill();
  rect(298, 6, 17, 23);
  rect(304, 4, 4, 1);

  noStroke();
  fill(255, 255, 255);

  if (battery <= 100 && battery >=75) {

    rect (302, 9, 10, 3);  
    rect (302, 14, 10, 3);
    rect (302, 19, 10, 3);
    rect (302, 24, 10, 3);
  }


  if (battery < 75 && battery >=50) {
    rect (302, 14, 10, 3);
    rect (302, 19, 10, 3);
    rect (302, 24, 10, 3);
  }


  if (battery < 50 && battery >=25) {
    rect (302, 19, 10, 3);
    rect (302, 24, 10, 3);
  }

  if (battery < 25 && battery >=18) {
    rect (302, 24, 10, 3);
  }
}