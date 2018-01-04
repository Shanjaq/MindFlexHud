import processing.serial.*;

/* Data CODE definitions */
int PARSER_BATTERY_CODE =       0x01;
int PARSER_POOR_SIGNAL_CODE =   0x02;
int PARSER_ATTENTION_CODE =     0x04;
int PARSER_MEDITATION_CODE =    0x05;
int PARSER_8BIT_RAW =           0x06;
int PARSER_RAW_MARKER =         0x07;
int PARSER_RAW_VALUE =          0x80;
int PARSER_EEG_POWER =          0x81;
int PARSER_ASIC_EEG_POWER =     0x83;
int PARSER_RRINTERVAL =         0x86;
int PARSER_SYNC_MARKER =        0xAA;

int parser_indices[] = 
{
  PARSER_BATTERY_CODE,
  PARSER_POOR_SIGNAL_CODE,
  PARSER_ATTENTION_CODE,
  PARSER_MEDITATION_CODE,
  PARSER_8BIT_RAW,
  PARSER_RAW_MARKER,
  PARSER_RAW_VALUE,
  PARSER_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_RRINTERVAL,
  PARSER_SYNC_MARKER
};

int parser_length[] = 
{
  1,
  1,
  1,
  1,
  1,
  1,
  2,
  32,
  25,
  2,
  1
};

String parser_names[] =
{
  "battery",
  "quality",
  "attention",
  "meditation",
  "8bit_raw",
  "raw_marker",
  "raw_value",
  "eeg_power",
  "asic_eeg_power",
  "rrinterval",
  "sync"
};


int bars = 10;
int[] bar_values = new int[bars];
int[] bar_parser_indices = 
{
  PARSER_ATTENTION_CODE,
  PARSER_MEDITATION_CODE,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER,
  PARSER_ASIC_EEG_POWER
};

int[] bar_max_values =
{
  100,
  100,
  65535,
  65535,
  65535,
  65535,
  65535,
  65535,
  65535,
  65535
};

int[] bar_colors =
{
  0xffffffff,
  0xffaaaaaa,
  0xff0000ff,
  0xff00ffff,
  0xff00ff00,
  0xffffff00,
  0xffff0000,
  0xffff00ff,
  0xfff0ff0f,
  0xfff0f0f0,
};

Serial port;
int bufLen = 256;
int packet_length_max = 173;
int packet_length_expected = 32;
byte[] pBuf = new byte[packet_length_max];
byte[] buffer = new byte[bufLen];

int readPos = 0;
int writePos = 0;

int maskIndex(int val, int len)  { return val & (len - 1); }
void push(byte val)  { assert(!full()); buffer[maskIndex(writePos++ & 0xffff, bufLen)] = val; }
byte shift()    { assert(!empty()); return buffer[maskIndex(readPos++ & 0xffff, bufLen)]; }
boolean empty()    { return readPos == writePos; }
boolean full()     { return dataLen() == bufLen; }
int dataLen()     { return (writePos - readPos) & 0xffff; }

int getIndex(int code)
{
  if (code >= 0x07)
  {
    for (int i = 0; i < parser_indices.length; i++)
    {
      if (parser_indices[i] == code)
      {
        return i;
      }
    }
  }
  else
  {
    for (int i = parser_indices.length - 1; i >= 0; i--)
    {
      if (parser_indices[i] == code)
      {
        return i;
      }
    }
  }
  return -1;
}

int RawData[] = new int[bars];;
float SmoothData[] = new float[bars];;
float LPF_Beta = 0.05; // 0<ß<1

int[][] result;

static int geoBufLen = 256;
float[][] geoBuffer;

int geoReadPos[] = new int[bars];
int geoWritePos[] = new int[bars];

void geoPush(float val, int channel)  { assert(!geoFull(channel)); geoBuffer[maskIndex(geoWritePos[channel]++ & 0xffff, geoBufLen)][channel] = val; }
float geoShift(int channel)    { assert(!geoEmpty(channel)); return geoBuffer[maskIndex(geoReadPos[channel]++ & 0xffff, geoBufLen)][channel]; }
boolean geoEmpty(int channel)    { return geoReadPos[channel] == geoWritePos[channel]; }
boolean geoFull(int channel)     { return geoDataLen(channel) == geoBufLen; }
int geoDataLen(int channel)     { return (geoWritePos[channel] - geoReadPos[channel]) & 0xffff; }

float orbitRadius= 300;
float phi= radians(1);
float theta= radians(1);

//Convert spherical coordinates into Cartesian coordinates
float cameraZ = orbitRadius*sin(phi)*cos(theta);
float cameraX = orbitRadius*sin(phi)*sin(theta);
float cameraY = orbitRadius*cos(phi);

PVector lastPos = new PVector();
PVector lastRot = new PVector();


void setup()
{
  size(640,480,P3D);
  surface.setAlwaysOnTop(true);
  port = new Serial(this, Serial.list()[3], 9600);
  geoBuffer = new float[geoBufLen][bar_values.length];
}
boolean sync1 = false;
boolean sync2 = false;
int pLen = 0;


void draw() 
{
  while ((port.available() > 0) && (!full())) {
    push(byte(port.read()));
  }
  boolean bDone = false;
  
  while (!bDone)
  {
    if (!empty())
    {
      byte b = shift();
      
      if (sync1 && sync2)
      {
        if (pLen == 0)
        {
          if ((b & 0xff) == packet_length_expected)
          {
            pLen = b & 0xff;
          }
          else
          {
            sync1 = false;
            sync2 = false;
          }
        }
        else
        {
          if (dataLen() >= pLen + 1)
          {
            int checksum = 0;
            for (int i = 0; i < pLen; i++)
            {
              pBuf[i] = b;
              checksum += b & 0xff;
              b = shift();
            }
            checksum = ~checksum & 0xff;
            if (checksum == (b & 0xff))
            {
              //print("OK");
              //println();
              
              int pPos = 0;
              while (pPos < pLen)
              {
                int mIndex = getIndex(pBuf[pPos] & 0xff);
                int mPos = 0;
                
                mPos++;
                if (mIndex != -1)
                {
                  while (mPos <= parser_length[mIndex])
                  {
                    for (int i = 0; i < bars; i++)
                    {
                      if (parser_indices[mIndex] == bar_parser_indices[i])
                      {
                        if ((parser_indices[mIndex]) == PARSER_ASIC_EEG_POWER)
                        {
                          for (int j = 0; j < 8; j++)
                          {
                            //((float *)data.data)[0] = lon; // uses data.data[0] ... data.data[3] //Arduino
                            bar_values[i] = 0;
                            for (int k = 0; k < 3; k++)
                            {
                              bar_values[i] += int(pBuf[pPos + (mPos + 1) + (j * 3) + k]) * (256 * (2-k));// & 0xffffff;
                            }
                            RawData[i] = bar_values[i];
                            SmoothData[i] = SmoothData[i] - (LPF_Beta * (SmoothData[i] - RawData[i]));
                            i++;
                          }
                          mPos += 24;
                        }
                        else
                        {
                          bar_values[i] = pBuf[pPos + mPos];
                          RawData[i] = bar_values[i];
                          SmoothData[i] = SmoothData[i] - (LPF_Beta * (SmoothData[i] - RawData[i]));
                        }
                      }
                    }
                    //print((pBuf[pPos + mPos] & 0xff) + ",");
                    mPos++;
                  }
                }
                else
                {
                  print("ERROR: Corrupt Packet");
                  println();
                  sync1 = false;
                  sync2 = false;
                  pLen = 0;
                }
                pPos = pPos + mPos;
              }
              bDone = true;
            }
            else
            {
              print("ERROR: " + checksum + " != " + (b & 0xff));
              println();
              for (int i = 0; i < pLen; i++)
              {
                print((pBuf[i] & 0xff) + " ");
              }
              println();
            }
            readPos += (pLen + 1);
            pLen = 0;
            sync1 = false;
            sync2 = false;
          }
        }
        bDone = true;
      }
      else
      {
        if ((b & 0xff) == PARSER_SYNC_MARKER)
        {
          if (sync1)
          {
            sync2 = true;
          }
          else 
          {
            sync1 = true;
          }
        }
        else
        {
          sync1 = false;
          sync2 = false;
        }
      }
    }
    else
    {
      bDone = true;
    }
  } 
  background(0);
  
  for (int i = 0; i < bars; i++)
  {
    fill(bar_colors[i]);
    rect((float(width/bars)*i), height - ((float)(SmoothData[i] / (double)bar_max_values[i]) * height), (float(width)/float(bars)), (float)(SmoothData[i] / (double)bar_max_values[i]) * height);
    geoPush((float)(SmoothData[i] / (double)bar_max_values[i]) * height, i);
    geoShift(i);
  }
  
  cameraZ = orbitRadius*sin(phi)*cos(theta);
  cameraX = orbitRadius*sin(phi)*sin(theta);
  cameraY = orbitRadius*cos(phi);
  
  pushMatrix();
  
  camera(cameraX, cameraY, cameraZ, 0, 0, 0.0, 
         0.0, 1.0, 0.0);
         
  noFill();
  stroke(bar_colors[3]);
  float stretch_x = 0.5;
  float stretch_y = 0.5;
  float stretch_z = 10;
  
  for (int i = 2; i < bar_values.length; i++)
  {
    beginShape();
    vertex(0, 0, i*stretch_z);
    for (int j = 0; j < geoBufLen; j++)
    {
      stroke(bar_colors[i]);
      vertex(j*stretch_x, -geoBuffer[j][i]*stretch_y, i*stretch_z);
    }
    vertex(geoBufLen*stretch_x, 0, i*stretch_z);
    endShape();
  }
  popMatrix();
}

void mousePressed()
{
  lastPos.x = mouseX;
  lastPos.y = mouseY;
}

void mouseReleased()
{
  lastRot.x = theta;
  lastRot.y = phi;
}

void mouseDragged()
{
  if (mouseButton == LEFT)
  {
      theta = lastRot.x + (radians(360 - (((float(mouseX)-lastPos.x)/(float(width)-lastPos.x)) * 360.0f)));
      phi = lastRot.y + (radians(((float(mouseY)-lastPos.y)/(float(height)-lastPos.y)) * 180.0f));
  }  
}


//The eight EEG powers are output in the following order:
//delta (0.5 - 2.75Hz)
//theta (3.5 - 6.75Hz)
//low-alpha (7.5 - 9.25Hz)
//high-alpha (10 - 11.75Hz)
//low-beta (13 - 16.75Hz)
//high-beta (18 - 29.75Hz)
//low-gamma (31 - 39.75Hz)
//mid-gamma (41 - 49.75Hz)