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

int maskIndex(int val)  { return val & (bufLen - 1); }
void push(byte val)  { assert(!full()); buffer[maskIndex(writePos++ & 0xffff)] = val; }
byte shift()    { assert(!empty()); return buffer[maskIndex(readPos++ & 0xffff)]; }
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

void setup()
{
  surface.setAlwaysOnTop(true);
  port = new Serial(this, Serial.list()[2], 9600);
  // Open a file and read its binary data 
//  byte buffer[] = loadBytes("C:\\test\\eeg_9600.log"); 
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
              //print((b & 0xff) + " ");
              pBuf[i] = b;
              checksum += b & 0xff;
              b = shift();
            }
            //print(": " + pLen);
            //println();
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
                //print(parser_names[mIndex] + ": ");
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
                //println();
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
            //println();
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
  
  noStroke();
  background(10);
  for (int i = 0; i < bars; i++)
  {
    fill(bar_colors[i]);
    rect((float(width/bars)*i), height - ((float)(SmoothData[i] / (double)bar_max_values[i]) * height), (float(width)/float(bars)), (float)(SmoothData[i] / (double)bar_max_values[i]) * height);
  }

  // Update the position of the shape
  //xpos = xpos + ( xspeed * xdirection );
  //ypos = ypos + ( yspeed * ydirection );
  
  // Test to see if the shape exceeds the boundaries of the screen
  // If it does, reverse its direction by multiplying by -1
  //if (xpos > width-rad || xpos < rad) {
  //  xdirection *= -1;
  //}
  //if (ypos > height-rad || ypos < rad) {
  //  ydirection *= -1;
  //}

  // Draw the shape
  //ellipse(xpos, ypos, rad, rad);
}

//The eight EEG powers are output in the following order:
//delta (0.5 - 2.75Hz): Too much: Brain injuries, learning problems, inability to think, severe ADHD; Too little: Inability to rejuvenate body, inability to revitalize the brain, poor sleep; Optimal: Immune system, natural healing, restorative / deep sleep
//theta (3.5 - 6.75Hz): Too much: ADHD, depression, hyperactivity, impulsivity, inattentiveness; Too little: Anxiety, poor emotional awareness, stress; Optimal: Creativity, emotional connection, intuition, relaxation
//low-alpha (7.5 - 9.25Hz): Too much: Daydreaming, inability to focus, too relaxed; Too little: Anxiety, high stress, insomnia, OCD; Optimal: Relaxation
//high-alpha (10 - 11.75Hz): Too much: Daydreaming, inability to focus, too relaxed; Too little: Anxiety, high stress, insomnia, OCD; Optimal: Relaxation
//low-beta (13 - 16.75Hz): Too much: Adrenaline, anxiety, high arousal, inability to relax, stress; Too little: ADHD, daydreaming, depression, poor cognition; Optimal: Conscious focus, memory, problem solving
//high-beta (18 - 29.75Hz): Too much: Adrenaline, anxiety, high arousal, inability to relax, stress; Too little: ADHD, daydreaming, depression, poor cognition; Optimal: Conscious focus, memory, problem solving
//low-gamma (31 - 39.75Hz): Too much: Anxiety, high arousal, stress; Too little: ADHD, depression, learning disabilities; Binding senses, cognition, information processing, learning, perception, REM sleep
//mid-gamma (41 - 49.75Hz): Too much: Anxiety, high arousal, stress; Too little: ADHD, depression, learning disabilities; Binding senses, cognition, information processing, learning, perception, REM sleep