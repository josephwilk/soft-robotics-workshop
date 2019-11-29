import processing.serial.*;
import themidibus.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port
int left;
int right;
int bottom;
int min;
int max;
int barLength = 2000;
MidiBus myBus;
int leftNote=0;
int rightNote=0;

void setup()
{
  String portName = "/dev/cu.usbmodem144301";
  myPort = new Serial(this, portName, 9600);
  myBus = new MidiBus(this, -1, 1);
  size(2010, 300);
}

void draw()
{
  if ( myPort.available() > 0){
    val = myPort.readStringUntil('\n');
  }
  if(val != null && val != "" && val != "\n" ){
    try{
    JSONObject json = parseJSONObject(val);
    left = json.getInt("l");
    right = json.getInt("r");
    bottom = json.getInt("b");
    min = json.getInt("min");
    max = json.getInt("max");
    min  = 5;
    max = 250;
    } catch(RuntimeException e){}
  }
max=250;
  fill(0);
  background(126);
  stroke(145,10,137);
  fill(255, 20, 147);

  //left = (int)random(0,250);
  //right = (int)random(0,250);
  //bottom = (int)random(0,250);

  float ratio = left;
  if(max > 0){
    ratio = ((float)left/(float)max);
  }
  rect(5,5,(int)(ratio*barLength), 90);

  textSize(32);
  if(val!=null && val != "\n"){
    if(left > 1 && leftNote !=1){
       println("note on");
      myBus.sendNoteOn(1,50,100);
      //delay(500);
      //myBus.sendNoteOff(1,50,10);
      leftNote=1;
    }
    if(leftNote == 1 && left > 0){
     myBus.sendControllerChange(1,100, int(map(left, 5, max, 127, 1  )));
    }
    if(left ==0 && leftNote==1){
      myBus.sendNoteOff(1,50,100);
       println("note off");
      leftNote=0;
    }
    fill(10, 10, 10);
    text("L: "+left+"mm", 10, 60);
  }

   ratio = right;
  if(max > 0){
    ratio = ((float)right/(float)max);
  }
  stroke(145, 100,137);
  fill(255,   100, 147);
  rect(5,    100,(int)(ratio*barLength), 90);
  if(val!=null && val != "\n"){
    if(right > 0 && rightNote !=1){
       myBus.sendNoteOn(1,52,10);
        rightNote=1;
    }
    fill(10,100, 10);
    fill(0);
     if(rightNote == 1 && right > 0){
       myBus.sendControllerChange(1,100, int(map(right, 5, max, 127, 1  )));
       //myBus.sendControllerChange(1,100, int(map(right, 0, 500, 127, 50)));
      //myBus.sendControllerChange(1,103, int(map(right, 0, 250, 1, 127)));
    }
    if(right == 0){
     myBus.sendNoteOff(1,50,10);
      rightNote=0;
    }
    text("R: "+right+"mm", 10, 150);
  }

  ratio = bottom;
  if(max > 0){
    ratio = ((float)bottom/(float)max);
  }

  stroke(145, 200,137);
  fill(255,   200, 147);
  rect(5,    200, (ratio*barLength), 90);
  if(val!=null && val != "\n"){
     if( bottom > 0){
       myBus.sendControllerChange(1,103, int(map(bottom, 5, max, 100, 1  )));
    }
    else{
       myBus.sendControllerChange(1,103, 0);
    }


    fill(10,200, 10);
    fill(0);
    text("B: "+bottom+"mm", 10, 250);
  }
}
