/*
Ball Balance Machine
Guiding a ball by tilting the surface based on video feedback

by David Thomasson
Feel free to use it as you wish.

Physical setup --------------
- camera is positioned so that ServoX is at top of frame and ServoY is at right of frame
- both Servos lift the board when they rotate clockwise
- servos rotate clockwise when angle is increased (ie. > 90) 

Arduino setup ---------------
- load with ServoFirmata (File > Examples > Firmata)
- servo controlling X (left-right) tilt is on pin9
- servo controlling Y (top-bottom) tilt is on pin10

*/

import processing.serial.*;
import cc.arduino.*;
import JMyron.*;

Arduino arduino;
JMyron m;

float ServoX_ang;  // servo angle in radians (+pi/2 => up (max clockwise),  -pi/2 => down (max counterclockwise))
float ServoY_ang;
int ServoX_flat = 93;  //servo 9
int ServoY_flat = 90;  //servo 10
int ServoX_val = ServoX_flat;
int ServoY_val = ServoY_flat;

int Xsize_pix = 320;  // horizontal size of video capture
int Ysize_pix = 240;  // vertical size of video capture
float Xsize_m = 0.385; // size of visible area (metres)
float Ysize_m = 0.29; // size of visible area (metres)

float board_centre_to_servo_centre_X = 0.237; // metres from centre of board to axis of servo
float board_centre_to_servo_centre_Y = 0.188; // metres from centre of board to axis of servo
float servo_arm_length_X = 0.0235; //length of servo arm from axis in metres
float servo_arm_length_Y = 0.0235; //length of servo arm from axis in metres

float board_angX;  // radians +ve when right of frame is up
float board_angY;  // radians +ve when bottom of frame is up
float board_angX_max = atan(servo_arm_length_X / board_centre_to_servo_centre_X); // max board angle
float board_angY_max = atan(servo_arm_length_Y / board_centre_to_servo_centre_Y); // max board angle

// used in PID control calcs
int error_X, error_Y; 
int previous_error_X, previous_error_Y;
float integral_X, integral_Y;
float derivative_X, derivative_Y;
float output_X, output_Y;
float Kp_X = 1;
float Ki_X = 0;
float Kd_X = 1;
float Ko_X = 4250;  //4250 is good for static target

float Kp_Y = 1;
float Ki_Y = 0;
float Kd_Y = 1;
float Ko_Y = 4750;  //4250 is good for static target

float t;    //time of current glob 
int Xpix;  //glob centre coord - current frame (measured in pixels from left edge)
int Ypix;  //glob centre coord - current frame (measured in pixels from left edge)
float Xm; //glob centre coord - current frame (measured in metres from centre of board)
float Ym; //glob centre coord - current frame (measured in metres from centre of board)
float Xnorm; //glob centre coord - current frame (ratio of posn in relation to target and edge of frame)
float Ynorm; //glob centre coord - current frame (ratio of posn in relation to target and edge of frame)

float t1;    //time of previous glob 
int X1pix; //glob centre coord - prev frame (measured in pixels from left edge)
int Y1pix; //glob centre coord - prev frame (measured in pixels from top edge)
float X1;  //glob centre coord - prev frame (measured in metres from centre of board)
float Y1;  //glob centre coord - prev frame (measured in metres from centre of board)

float t2;    //time of glob 2 frames ago
int X2pix; //glob centre coord - prev frame (measured in pixels from left edge)
int Y2pix; //glob centre coord - prev frame (measured in pixels from top edge)
float X2;  //glob centre coord - 2 frames ago (measured in metres from centre of board)
float Y2;  //glob centre coord - 2 frames ago (measured in metres from centre of board)

float velX; //measured in metres/sec - +ve going to right of frame)
float velY; //measured in metres/sec - +ve going to bottom of frame)
float velX1; //prev vel, measured in metres/sec - +ve going to right of frame)
float velY1; //prev vel, measured in metres/sec - +ve going to bottom of frame)
float accX; //measured in metres/sec/sec - +ve going to right of frame)
float accY;  //measured in metres/sec/sec - +ve going to bottom of frame)

int Xtarget = 160; //pixels from left edge
int Ytarget = 120; //pixels from top edge

boolean auto = false;  //used to turn off autobalancing
boolean oscillate = false;  //used to turn off autobalancing
boolean firstrun = true; //used for velocity calcs
boolean help = false; //used to display help info
boolean follow_circle = false; //used for shape following - circle
boolean follow_lemniscape = false; //used for shape following - infinity symbol
boolean follow_mouse = false; //used for mouse following
boolean datalog = false; //used for data logging to file

float follow_time; //used to check follow step interval
int radius = 60; //pixel radius for circular follow shape
float period = 10; //seconds per revolution
float angle; //angular position of target
float ti;  // time since starting a shape follow (secs)
float afoc = 90;  // focus dist for lemniscape (pixels)
float lem_period = 10;  // focus dist for lemniscape (pixels)

float freq = 0.5;  // board oscillation cycles/sec
float beta = 0;  // current angle in oscillation cycle
float osc_start;  // start time of the oscillation sequence
float osc_ang = 0.06;  // board angle of oscillation

int K_digits = 4;
float Kfactor = 0;
    
// for writing data to file
PrintWriter output;

//----------------------------------------------------------------------------
void setup(){
  size(Xsize_pix,Ysize_pix);
  
  m = new JMyron();
  m.start(Xsize_pix,Ysize_pix);
  m.findGlobs(1);  // 1 is on, 0 is off
  
  PFont font;
  font = loadFont("ArialMT-10.vlw");
  textFont(font);
  
  arduino = new Arduino(this, Arduino.list()[0], 57600);
  
  // Create a new file in the sketch directory
  output = createWriter("ball_data.txt");

}

//==============================================================================
void draw(){
  background(0);  // used only to clear the text from the previous frame
  
    if (keyPressed) {
      if (key < 'A' && key > '/') {
        Kfactor += (key-48) * pow(10, (K_digits-1));
        K_digits --;
      }
      if (K_digits == 0) {
        Ko_Y = Kfactor;
        K_digits = 4;
        Kfactor = 0;
      }
    }
    
  // experiment with these values to get the best tracking 
  m.trackColor(0,0,0,200);
 
  m.update();
  int[] img = m.image();
  
  //first draw the camera view onto the screen
  loadPixels();
  m.imageCopy(pixels);

  updatePixels();
  
  noFill();
  int[][] a;


  //-------------------------------------------------------------------------

  //track glob boxes
  a = m.globBoxes();
  stroke(255,0,0);
  if (a.length > 0){
    int[] b = a[0];    //look at the first glob only (0)

    //draw bounding boxes of globs
    noFill();
    rect(b[0], b[1], b[2], b[3]);
  }

  
  //-------------------------------------------------------------------------
  
  t2 = t1;
  t1 = t;
  t = millis();
  
  //set 2 frames ago posn
  X2pix = X1pix;
  Y2pix = Y1pix;
  X2 = X1;
  Y2 = Y1;
  //set 1 frame ago posn
  X1pix = Xpix;
  Y1pix = Ypix;
  X1 = Xm;
  Y1 = Ym; 


  // track glob centres
  a = m.globCenters();
  
  if (a.length > 0){  //if it found a glob -----

    int[] p = a[0];    //look at the first glob only (0)
    Xpix = p[0];
    Ypix = p[1];

    //draw center points of globs
    //stroke(255,255,0);    
    //point(X,Y);

    // writes the coord of the glob centre beside glob
    //fill(50);
    //text("("+X+","+Y+")", X + 20, Y + 20);

  } else { //if it didn't find a glob (ie. moving too fast) -----
    //try to predict where it will be
    
    Xpix = int(X1pix + float(X1pix - X2pix) * (t - t1) / (t1 - t2));
    Ypix = int(Y1pix + float(Y1pix - Y2pix) * (t - t1) / (t1 - t2));
    

  }

    float fXpix = float(Xpix);
    float fYpix = float(Ypix);
    
    Xm = ((fXpix/Xsize_pix) * Xsize_m) - (Xsize_m / 2);  //metres from centre of board
    Ym = ((fYpix/Ysize_pix) * Ysize_m) - (Ysize_m / 2);  //metres from centre of board
    
    velX1 = velX;
    velY1 = velY;
    
    velX = (Xm - X1)/((t - t1)/1000);  // m/sec
    velY = (Ym - Y1)/((t - t1)/1000);  // m/sec

  
  //-------------------------------------------------------------------------
  // calculate servo positions

  // *** target pattern     *****
  // circle
  if (follow_circle == true) {
      
      ti = (millis() - follow_time)/1000;
      angle = 2 * PI * ti / period;
  
      Xtarget = int((Xsize_pix / 2) + (radius * cos(angle)));
      Ytarget = int((Ysize_pix / 2) - (radius * sin(angle)));    
    
  // lemniscape (infinity symbol)
  } else if (follow_lemniscape == true) {
      
      ti = 2 * PI / lem_period * (millis() - follow_time)/1000;

      Xtarget = int((Xsize_pix / 2) + (afoc * cos(ti)) / (1 + pow(sin(ti),2)));
      Ytarget = int((Ysize_pix / 2) + (afoc * sin(ti) * cos(ti)) / (1 + pow(sin(ti),2)));
 
  // mouse follow
  } else if (follow_mouse == true) {
      Xtarget = mouseX;
      Ytarget = mouseY;
  }
  
  // *** calculate servo positions
  
  if (auto == true) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ServoX ------------------------
    // PID control
    // previous_error & integral are reset to 0 in keypressed()
    error_X = Xtarget - Xpix;
    integral_X = integral_X + (error_X * (t - t1)/1000);
    derivative_X = (error_X - previous_error_X) / ((t - t1)/1000);

    output_X = (Kp_X*error_X) + (Ki_X*integral_X) + (Kd_X*derivative_X);
    previous_error_X = error_X;

    board_angX = constrain((-output_X / Ko_X), -board_angX_max, board_angX_max); // +ve when right (X=320) edge is up
    
    // this is for servoX located at right (X=320) edge (ServoX_ang = +ve when board_angY = +ve)
    ServoX_ang = asin((board_centre_to_servo_centre_X * tan(board_angX)) / sqrt(pow(servo_arm_length_X, 2) + pow(servo_arm_length_X * tan(board_angX), 2))) - board_angX;
    ServoX_val = int(map(ServoX_ang, -PI/2, PI/2, (ServoX_flat - 90), (90 + ServoX_flat)));     

    // ServoY ------------------------
    error_Y = Ytarget - Ypix;
    integral_Y = integral_Y + (error_Y * (t - t1)/1000);
    derivative_Y = (error_Y - previous_error_Y) / ((t - t1)/1000);

    output_Y = (Kp_Y*error_Y) + (Ki_Y*integral_Y) + (Kd_Y*derivative_Y);
    previous_error_Y = error_Y;

    board_angY = constrain((-output_Y / Ko_Y), -board_angY_max, board_angY_max); // +ve when Y=240 is up
    
    // this is for servoY located at top (Y=0) edge (ServoY_ang = -ve when board_angY = +ve)
    ServoY_ang = - asin((board_centre_to_servo_centre_Y * tan(board_angY)) / sqrt(pow(servo_arm_length_Y, 2) + pow(servo_arm_length_Y * tan(board_angY), 2))) - board_angY;
    ServoY_val = int(map(ServoY_ang, -PI/2, PI/2, (ServoY_flat - 90), (90 + ServoY_flat)));       

    // adjust servos ---------------------
      arduino.analogWrite(9, ServoX_val);
      arduino.analogWrite(10, ServoY_val);

  } else if (oscillate == true) { //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

      beta = ((millis() - osc_start)/1000)/(1/freq) * 2 * PI;  //angle of oscillation cycle
      
      board_angX = 2 * asin(cos(beta) * sin(osc_ang / 2));
      board_angY = 2 * asin(sin(beta) * sin(osc_ang / 2));
  
      ServoX_ang = asin(((board_centre_to_servo_centre_X / servo_arm_length_X) - 1) * tan(board_angX));
      ServoY_ang = -asin(((board_centre_to_servo_centre_Y / servo_arm_length_Y) - 1) * tan(board_angY));

      ServoX_val = int(map(ServoX_ang, -PI/2, PI/2, (ServoX_flat - 90), (90 + ServoX_flat)));     
      ServoY_val = int(map(ServoY_ang, -PI/2, PI/2, (ServoY_flat - 90), (90 + ServoY_flat)));
  
      // adjust servos ---------------------
      arduino.analogWrite(9, ServoX_val);
      arduino.analogWrite(10, ServoY_val);
      
  }
  
  //-------------------------------------------------------------------------
  //data display & recording
  fill(255, 0, 0);
  // show position data
  String s = frameRate+
          "\nt = "+int(t)+
          "\n"+Xpix+","+Ypix+
          "\nServoX_val = "+ServoX_val+
          "\nServoY_val = "+ServoY_val+
          "\nlog = "+datalog+
          "\nKo_Y = "+Ko_Y+
          "\nK_digits = "+K_digits+
          "\nKfactor = "+Kfactor;
  text(s, 5, 15);    

  //show help data
  if (help == true) {
    String helpstring = "SPACE       = set servos flat"+
                      "\nZ           = set servos to min"+
                      "\nX           = set servos to max"+
                      "\nQ           = raise ServoX"+
                      "\nA           = lower ServoX"+
                      "\nW           = raise ServoY"+
                      "\nS           = lower ServoY"+
                      "\nE           = set flat posns"+
                      "\nK           = camera settings"+
                      "\nC           = tracking colour"+
                      "\nclick       = reset target position"+
                      "\nENTER       = toggle autobalancing"+
                      "\nG           = toggle CIRCLE following"+
                      "\nI           = toggle INFINITY following"+
                      "\nM           = toggle mouse following"+
                      "\nO           = toggle oscillate"+
                      "\nL           = toggle data logging"+                      
                      "\n0-9         = enter Kfactor"+        
                      "\nY           = end program"+
                      "\nH           = toggle help"+
                      "\nServoX_flat = "+ServoX_flat+
                      "\nServoY_flat = "+ServoY_flat;              

    text(helpstring, 75, 15);  
  }
  
  //show target posn
  line(Xtarget, Ytarget-5, Xtarget, Ytarget+5);
  line(Xtarget-5, Ytarget, Xtarget+5, Ytarget);
  text("("+Xtarget+","+Ytarget+")", Xtarget+5, Ytarget+10);
  
  if (datalog == true) {
    
    saveFrame("#####.png");
    
    // Write the coordinate to a file with a
    // "\t" (TAB character) between each entry
    output.println(t+"\t"+Xtarget+"\t"+Xpix+"\t"+Xm+"\t"+X1+"\t"+X2+"\t"+velX+"\t"+board_angX+"\t"+ServoX_ang+"\t"+ServoX_val+"\t"+
                          Ytarget+"\t"+Ypix+"\t"+Ym+"\t"+Y1+"\t"+Y2+"\t"+velY+"\t"+board_angY+"\t"+ServoY_ang+"\t"+ServoY_val+"\t"+
                          output_X+"\t"+error_X+"\t"+integral_X+"\t"+derivative_X);
  }


}

// =============================================================================
void keyPressed()
{
  if(key == ' ') {
    //set servos to level
    ServoX_val = ServoX_flat;
    ServoY_val = ServoY_flat;
    arduino.analogWrite(9, ServoX_val);
    arduino.analogWrite(10, ServoY_val);
  } else if (key == 'Z' || key == 'z'){
    //set servos to minimum
    ServoX_val = 0;
    ServoY_val = 0;
    arduino.analogWrite(9, ServoX_val);
    arduino.analogWrite(10, ServoY_val);
  } else if (key == 'X' || key == 'x'){
    //set servos to maximum
    ServoX_val = 179;
    ServoY_val = 179;
    arduino.analogWrite(9, ServoX_val);
    arduino.analogWrite(10, ServoY_val);
  } else if (key == 'Q' || key == 'q'){
    //raise X servo (servo 9)
    ServoX_val = constrain(ServoX_val + 1, 0, 179);
    arduino.analogWrite(9, ServoX_val);
  } else if (key == 'A' || key == 'a'){
    //lower X servo (servo 9)
    ServoX_val = constrain(ServoX_val - 1, 0, 179);
    arduino.analogWrite(9, ServoX_val);
  } else if (key == 'W' || key == 'w'){
    //raise Y servo (servo 10)
    ServoY_val = constrain(ServoY_val + 1, 0, 179);
    arduino.analogWrite(10, ServoY_val);
  } else if (key == 'S' || key == 's'){
    //lower Y servo (servo 10)
    ServoY_val = constrain(ServoY_val - 1, 0, 179);
    arduino.analogWrite(10, ServoY_val);
  } else if (key == 'E' || key == 'e'){
    //set the current Servo settings as the flat point
    ServoX_flat = ServoX_val;
    ServoY_flat = ServoY_val;
  } else if (key == 'K' || key == 'k'){
    //camera settings
    m.settings();
  } else if (key == 'H' || key == 'h'){
    //toggle Help display
    help = !help;
  } else if (key == ENTER){
    //toggle the autobalancing
    auto = !auto;
    oscillate = false;
    previous_error_X = 0;
    integral_X = 0;
    previous_error_Y = 0;
    integral_Y = 0;    
  } else if (key == 'G' || key == 'g'){
    //toggle the shape following - CIRCLE
    follow_circle = !follow_circle;
    follow_time = millis();
    follow_lemniscape = false;
    follow_mouse = false;
    angle = 0;
    previous_error_X = 0;
    integral_X = 0;
    previous_error_Y = 0;
    integral_Y = 0;   
  } else if (key == 'I' || key == 'i'){
    //toggle the shape following - LEMNISCAPE
    follow_lemniscape = !follow_lemniscape;
    follow_time = millis();
    follow_circle = false;
    follow_mouse = false;
    angle = 0;
    previous_error_X = 0;
    integral_X = 0;
    previous_error_Y = 0;
    integral_Y = 0;       
  } else if (key == 'M' || key == 'm'){
    //toggle the shape following
    follow_mouse = !follow_mouse;
    follow_circle = false;
    follow_lemniscape = false;
    previous_error_X = 0;
    integral_X = 0;
    previous_error_Y = 0;
    integral_Y = 0;      
  } else if (key == 'O' || key == 'o'){
    //toggle the board oscillation
    oscillate = !oscillate;
    osc_start = millis();
    auto = false;
  } else if (key == 'L' || key == 'l'){
    //toggle the data logging
    datalog = !datalog;
  } else if (key == 'Y' || key == 'y'){
    output.flush(); // Write the remaining data
    output.close(); // Finish the file
    
    // adjust servos to flat
    arduino.analogWrite(9, ServoX_flat);
    arduino.analogWrite(10, ServoY_flat);
    
    exit(); // Stop the program

  }
}

//=============================================================================
void mousePressed() {
  //reset target position
  Xtarget = mouseX;
  Ytarget = mouseY;
  follow_circle = false;
  follow_lemniscape = false;
  follow_mouse = false;

}

//=============================================================================
public void stop(){
  m.stop();
  super.stop();
}

