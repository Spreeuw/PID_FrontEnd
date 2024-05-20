/********************************************************
 * Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 *
 * This application is designed to interface with an
 * arduino running the PID Library.  From this Control
 * Panel you can observe & adjust PID performance in 
 * real time
 *
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 * 
 ********************************************************/
 /* Updates /Infos to v_04:
 - In line 102, set the index of your tty list to the one your sending Arduino is connected to. 
 The Processing console lists all Serial ports at the very beginning.
 
 - making Setpoint/Input/Output changeable in the UI without Crashes
 
 - replaced a couple of valueLabel with getValueLabel for latest P5 compatability
 
 */
 
 

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1280;      // set the size of the 
int windowHeight = 800;     // form

float inScaleMin = 0.0;       // set the Y-Axis Min
float inScaleMax = 50.0;    // and Max for both
float inGridSpacing = 5.0; 

float outScaleMin = -100.0;      // the top and 
float outScaleMax = 100.0;    // bottom trends
float outGridSpacing = 25.0;

int windowSpan = 3600000;    // number of mS into the past you want to display
int refreshRate = 1000;      // how often you want the graph to be reDrawn;

//float displayFactor = 1; //display Time as Milliseconds
//float displayFactor = 1000; //display Time as Seconds
float displayFactor = 60000; //display Time as Minutes
int timeGridSpacing = 5;

boolean hasPrintedData = false;

String outputFileName = ""; // if you'd like to output data to 
// a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = (windowSpan / refreshRate)+2; // fencepost + 1 spare for smooth transition
int[] inputData = new int[arrayLength];     //we might not need them this big, but
int[] setpointData = new int[arrayLength];  // this is worst case
int[] outputData = new int[arrayLength];


int inputTop = 35;
int inputHeight = int((windowHeight-90)*2/3);
int outputTop = inputHeight+70;
int outputHeight = int((windowHeight-90)*1/3);

int ioLeft = 170;
int ioWidth = windowWidth-ioLeft-50;
int ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 12;

int nPoints = 0;
int dataStartTime = 0;
int startTime = 0;

float Input, Setpoint, Output;

boolean madeContact =false;
boolean justSent = true;

Serial myPort;

ControlP5 cp5;
controlP5.Button AMButton, DRButton;
controlP5.Textlabel AMLabel, AMCurrent, InLabel, 
OutLabel, SPLabel, PLabel, 
ILabel, DLabel,DRLabel, DRCurrent;
controlP5.Textfield SPField, InField, OutField, 
PField, IField, DField;

PFont AxisFont, TitleFont; 

void setup()
{
  frameRate(30);
  size(2000,2000); // ControlP5 only detects events within the size scope, so we need a size bigger than the surface size
  surface.setSize(windowWidth, windowHeight);

  String[] portNames = Serial.list();

  cp5 = new ControlP5(this); 
  
  // Initialize the various Buttons, Labels, and Text Fields we'll be using
  
  // Serial port selector
  cp5.addScrollableList("serial ports").setPosition(10, 10).setWidth(130).setBarHeight(20);
  for(int i = 0 ; i < portNames.length; i++) cp5.get(ScrollableList.class, "serial ports").addItem(portNames[i], i);
  cp5.get(ScrollableList.class, "serial ports").setValue(0).close();

  // Automatic/Manual toggle
  AMButton = cp5.addButton("Toggle_AM").setPosition(10, 75).setSize(60,20);
  AMLabel = cp5.addTextlabel("AM","Manual",12,97);
  AMCurrent = cp5.addTextlabel("AMCurrent","Manual",80,80);

  // Setpoint setter
  SPField = cp5.addTextfield("SetpointSetter",10,125,60,20).setInputFilter(ControlP5.FLOAT);
  SPLabel = cp5.addTextlabel("SP","3",80,131);
  
  // Input setter
  InField = cp5.addTextfield("InputSetter",10,175,60,20).setInputFilter(ControlP5.FLOAT);
  InLabel = cp5.addTextlabel("In","1",80,181);

  // Output setter
  OutField = cp5.addTextfield("OutputSetter",10,225,60,20).setInputFilter(ControlP5.FLOAT);
  OutLabel=cp5.addTextlabel("Out","2",80,231);
  
  // Kp  
  PField = cp5.addTextfield("Kp (Proportional)",10,300,60,20);
  PLabel = cp5.addTextlabel("P","4",80,305);

  // Ki
  IField = cp5.addTextfield("Ki (Integral)",10,350,60,20);
  ILabel = cp5.addTextlabel("I","5",80,355);

  // Kd
  DField = cp5.addTextfield("Kd (Derivative)",10,400,60,20);
  DLabel = cp5.addTextlabel("D","6",80,405);

  // Toggle Direct/Reverse
  DRButton = cp5.addButton("Toggle_DR").setPosition(10, 450).setSize(60,20);
  DRLabel = cp5.addTextlabel("DR","Direct",12,472);
  DRCurrent = cp5.addTextlabel("DRCurrent","Direct",80,455);

  cp5.addButton("Send_To_Arduino").setPosition(10, 500).setSize(120,20);
  cp5.addButton("Clear data").setPosition(10, 525).setSize(120,20);
  
  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  if (outputFileName!="") output = createWriter(outputFileName);
  
}

void SetpointSetter(String v) {
  Setpoint = float(v);
}

void InputSetter(String v) {
  Input = float(v);
}

void OutputSetter(String v) {
  Output = float(v);
}

void draw()
{
  background(200);
  drawGraph();
  drawButtonArea();
}

void drawGraph()
{
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop, ioWidth, inputHeight);
  rect(ioLeft, outputTop, ioWidth, outputHeight);
  stroke(210);

  //Section Titles
  textFont(TitleFont);
  textSize(24);
  fill(0,35,102);
  text("Input / Setpoint",(int)ioLeft,(int)inputTop-8);
  text("Output",(int)ioLeft,(int)outputTop-8);


  //GridLines and Titles
  textFont(AxisFont);
  //horizontal grid lines
  for (float i = ceil(inScaleMin / inGridSpacing) * inGridSpacing; i <= inScaleMax; i += inGridSpacing) {
    int gridStrokeColor = i != 0.0 ? 210 : 0;
    stroke(gridStrokeColor);
    float y = (inputTop + inputHeight) - (i - inScaleMin) / (inScaleMax - inScaleMin) * inputHeight;
    line(ioLeft + 1, y, ioRight - 1, y);
    text(str(i), ioRight + 5, y + 4);
  }

  for (float i = ceil(outScaleMin / outGridSpacing) * outGridSpacing; i <= outScaleMax; i += outGridSpacing) {
    int gridStrokeColor = i != 0.0 ? 210 : 127;
    stroke(gridStrokeColor);
    float y = (outputTop + outputHeight) - (i - outScaleMin) / (outScaleMax - outScaleMin) * outputHeight;
    line(ioLeft + 1, y, ioRight - 1, y);
    text(str(i), ioRight + 5, y + 4);
  }

  
  //vertical grid lines and TimeStamps
  if(nPoints > 0){   
    float intervalMs = displayFactor * timeGridSpacing;

    int now = millis();
    for(int i = startTime; i < now; i += (int)intervalMs){
      if(i >= max(dataStartTime, now - windowSpan)){
        int gridLineX = getInputPosX( i );
        int gridStrokeColor = i-startTime != 0.0 ? 210 : 127;
        stroke(gridStrokeColor);
        line(gridLineX,inputTop+1,gridLineX,inputTop+inputHeight-1);
        line(gridLineX,outputTop+1,gridLineX,outputTop+outputHeight-1);
        text(str((i-startTime) / displayFactor),gridLineX,outputTop+outputHeight+10);        
      }
    }
  }
  // add the latest data to the data Arrays, storing the calculated Y coordinates
  if(millis() > nextRefresh && madeContact) {
    if (nPoints == 0) {
      nextRefresh = dataStartTime = startTime = millis()+refreshRate;
    } else {
      nextRefresh = nextRefresh + refreshRate;
    }

    // If the buffer is full, shift all elements to the left to free the last position
    if (nPoints == arrayLength) {
        for (int i = 0; i < arrayLength - 1; ++i) {
            inputData[i] = inputData[i + 1];
            setpointData[i] = setpointData[i + 1];
            outputData[i] = outputData[i + 1];
        }
        dataStartTime += refreshRate;
    } else {
      nPoints++;
    }

    inputData[nPoints-1] = getInputPosY(Input);
    setpointData[nPoints-1] = getInputPosY(Setpoint);
    outputData[nPoints-1] = getOutputPosY(Output);
    
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);
  if(nPoints > 1){
    for(int i=0; i<nPoints-1; i++)
    {
      int X1 = getInputPosX( dataStartTime+(float(i)*refreshRate) );
      int X2 = getInputPosX( dataStartTime+(float(i+1)*refreshRate) );
      //DRAW THE INPUT
      stroke(255,0,0);
      line(X1, inputData[i], X2, inputData[i+1]);
  
      //DRAW THE SETPOINT
      stroke(0,255,0);
      line(X1, setpointData[i], X2, setpointData[i+1]);
  
      //DRAW THE OUTPUT
      stroke(0,0,255);
      line(X1, outputData[i], X2, outputData[i+1]);
    }
  }
  strokeWeight(1);
  
  // cover up our operation
  fill(0,0);
  stroke(127);
  rect(ioLeft, inputTop, ioWidth, inputHeight);
  rect(ioLeft, outputTop, ioWidth, outputHeight);
 
}

void drawButtonArea()
{
  stroke(100);
  fill(100);
  rect(0, 0, ioLeft-20, windowHeight);
}

void Toggle_AM() {
  if(AMLabel.getValueLabel().getText()=="Manual") 
  {
    AMLabel.setValue("Automatic");
  }
  else
  {
    AMLabel.setValue("Manual");   
  }
}


void Toggle_DR() {
  if(DRLabel.getValueLabel().getText()=="Direct") 
  {
    DRLabel.setValue("Reverse");
  }
  else
  {
    DRLabel.setValue("Direct");   
  }
}

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduino()
{
  float[] toSend = new float[6];

  toSend[0] = float(SPField.getText());
  toSend[1] = float(InField.getText());
  toSend[2] = float(OutField.getText());
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  Byte a = (AMLabel.getValueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.getValueLabel().getText()=="Direct")?(byte)0:(byte)1;
  myPort.write(a);
  myPort.write(d);
  myPort.write(floatArrayToByteArray(toSend));
  justSent=true;
} 


byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) 
  {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}


//take the string the arduino sends us and parse it
void serialEvent(Serial myPort)
{
  String read = myPort.readStringUntil(10);
  if(outputFileName!="") output.print(str(millis())+ " "+read);
  String[] s = split(read, " ");

  if (s.length ==9)
  {
    Setpoint = float(s[1]);           // * pull the information
    Input = float(s[2]);              //   we need out of the
    Output = float(s[3]);             //   string and put it
    SPLabel.setValue(s[1]);           //   where it's needed
    InLabel.setValue(s[2]);           //
    OutLabel.setValue(trim(s[3]));    //
    PLabel.setValue(trim(s[4]));      //
    ILabel.setValue(trim(s[5]));      //
    DLabel.setValue(trim(s[6]));      //
    AMCurrent.setValue(trim(s[7]));   //
    DRCurrent.setValue(trim(s[8]));
    if(justSent)                      // * if this is the first read
    {                                 //   since we sent values to 
      SPField.setText(trim(s[1])).setLock(false);    //   the arduino,  take the
      InField.setText(trim(s[2]));    //   current values and put
      OutField.setText(trim(s[3]));   //   them into the input fields
      PField.setText(trim(s[4]));     //
      IField.setText(trim(s[5]));     //
      DField.setText(trim(s[6]));     //
     // mode = trim(s[7]);              //
      AMLabel.setValue(trim(s[7]));         //
      //dr = trim(s[8]);                //
      DRCurrent.setValue(trim(s[8]));         //
      justSent=false;                 //
    }                                 //

    if(!madeContact) madeContact=true;
  }
}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    // handle group events
  } 
  else if (theEvent.isController()) {
    String eventName = theEvent.getController().getName();
    switch(eventName) {
      case "serial ports": 
        int selected = round(+theEvent.getController().getValue());
        String portName = cp5.get(ScrollableList.class, "serial ports").getItem(selected).get("name").toString();
        setSerialPort(portName);
        break;
      case "Clear data": 
        clearData();
        break;
    }
  }
}

void setSerialPort(String portName) {
  println("Switch to serial port : "+ portName);
  //check if there's a serial port open already, if so, close it
  if(myPort != null){
    myPort.stop();
    myPort = null;
  }
  //open the selected port
  try{
    clearData();
    myPort = new Serial(this,portName,9600);
    myPort.bufferUntil(10);
  }catch(Exception e){
    System.err.println("Error opening serial port " + portName + "try selecting a different port in the application");
  }
}  

void clearData() {
  inputData = new int[arrayLength];
  setpointData = new int[arrayLength];
  outputData = new int[arrayLength];
  nPoints = 0;
  dataStartTime =  millis();
  startTime = millis();
}

int getInputPosX (float value) {
  int now = millis();
  int graphWidthMillis = constrain(now-dataStartTime,0,windowSpan);
  int graphWidthPx = int( (float) graphWidthMillis * ioWidth / (float) windowSpan );
  int graphStart = max(dataStartTime,now-windowSpan);
  int relativePos = getGraphPos((float) value, (int) graphWidthPx, (float) graphStart, (float) now);
  return ioRight - graphWidthPx + relativePos;
}

int getInputPosY (float value) {
  int relativePos = getGraphPos( (float) value, (int) inputHeight, (float) inScaleMin, (float) inScaleMax);
  return inputTop + inputHeight - relativePos;
}

int getOutputPosY (float value) {
  int relativePos = getGraphPos( (float) value, (int) outputHeight, (float) outScaleMin, (float) outScaleMax);
  return outputTop + outputHeight - relativePos;
}

int getGraphPos(float value, int graphSize, float graphMin, float graphMax) {
    float range = graphMax - graphMin;
    float relativePosition = (value - graphMin) / range;
    int graphPos = int(relativePosition * graphSize);
    graphPos = constrain(graphPos, 0, graphSize);
    return graphPos;
}
