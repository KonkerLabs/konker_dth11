//for LED status
#include <Ticker.h>
Ticker _bticker;

//initial values to identify the ESP by blinking
int  _bblinksTimes=1;
int  _bledWaitOffPeriod=4;

//auxiliar variables to blink id
int  _bledBlinksCount=0;
int  _bledOffState=1;
int  _bledWaitCount=0;
int _STATUS_LED=2;



void  _btick()
{
  //toggle state
  int state = digitalRead(_STATUS_LED);  // get the current state of GPIO1 pin
  digitalWrite(_STATUS_LED, !state);     // set pin to the opposite state
  if(_bledOffState==1){
    _bledBlinksCount=_bledBlinksCount+state;
  }else{
    _bledBlinksCount=_bledBlinksCount+!state;
  }

}


//blink  function
void  _bblink()
{
  if( _bledBlinksCount<_bblinksTimes){
    //tick and count +1 blink
     _btick();

  }else{
    //set led off
    digitalWrite(_STATUS_LED, _bledOffState);

    if( _bledWaitCount< _bledWaitOffPeriod){
      // +1 count to keep led off
       _bledWaitCount++;
    }else{
      //reached ledWaitOffPeriod period.
      // Reset variables start all over ain

      //set led off
      digitalWrite(_STATUS_LED,_bledOffState);

      _bledWaitCount=0;
      _bledBlinksCount=0;
    }
  }
}


//startBlinkID  function
//set ID zero to stop
void startBlinkID(int ID)
{
    if(ID>0){
      _bblinksTimes=ID;
      _bticker.attach(0.3,  _bblink);
    }else{
      _bticker.detach();
    }

}



char const *verifyMyblinkID(String ssidName){

  //set led pin as output
  pinMode(_STATUS_LED, OUTPUT);

  Serial.println("scan start");
  int ESPMaxnum=0;
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  delay(2000);

  if (n == 0)
    Serial.println("no networks found");
  else
  {
    String ssidTempName=ssidName;
    Serial.println("same networks found");
    for (int i = 0; i < n; ++i){
      String stringOne = WiFi.SSID(i).c_str();
      int foundIndex = stringOne.indexOf(ssidName);
      if (foundIndex==0){
        int ESPnum=WiFi.SSID(i).substring(ssidTempName.length()+1, WiFi.SSID(i).length()).toInt();
        if(ESPMaxnum<ESPnum){
          ESPMaxnum=ESPnum;
        }
      }
      delay(10);
    }

    ssidName=ssidName +"_"+(ESPMaxnum+1);
  }

  // startBlickID
  startBlinkID(ESPMaxnum+1);
  return ssidName.c_str();
}



char const *blinkMyID(String ssidName){
  int tentativas=10;
  WiFi.begin();//tenta connectar direto
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED && tentativas>0)
  {
    delay(1000);
    Serial.print(".");
    tentativas--;
  }
  if (WiFi.status() != WL_CONNECTED){
    Serial.println("");
    ssidName= verifyMyblinkID(ssidName);
    Serial.println("Not connected, broadcasting SSID for configuration.. "  + ssidName);
  }else{
    startBlinkID(0);// pass zero to stop
    //keep LED on
    digitalWrite(_STATUS_LED, LOW);

  }
  return ssidName.c_str();

}



char const *blinkMyID(String ssidName, int statusLed){
  //set led pin as output
  pinMode(statusLed, OUTPUT);

  return blinkMyID(ssidName);
}
