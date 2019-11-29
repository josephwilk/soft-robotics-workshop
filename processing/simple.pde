import processing.serial.*;

Serial myPort;       // Create object from Serial class
String data = "";    // Data received from the serial port

void setup()
{
  String portName = "/dev/cu.usbmodem144301";
  myPort = new Serial(this, portName, 9600);
  size(2010, 300);
}

void draw()
{
  if ( myPort.available() > 0){
    data = myPort.readStringUntil('\n');
  }
  if(data != null && data != "\n" ){
  int distance = int(data);

  fill(0);
  background(126);
  stroke(145,10,137);
  fill(255, 20, 147);

  rect(5,5,distance, 90);

  textSize(32);
  fill(10, 10, 10);
  text(""+distance+"mm", 10, 60);
  }
}
