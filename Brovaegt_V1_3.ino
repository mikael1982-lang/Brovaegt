#include <WiFi.h>
#include <WebServer.h>
#include "HX711.h"

#define DOUT 4
#define SCK  5

// RFID module UART (RX=16, TX=17)
const int RFID_RX_PIN = 16;
const int RFID_TX_PIN = 17;
const byte READ_MULTI_CMD[10] = {0xAA, 0x00, 0x27, 0x00, 0x03, 0x22, 0xFF, 0xFF, 0x4A, 0xDD};
const size_t RFID_MAX_FRAME_LENGTH=64;
const unsigned long RFID_TAG_TIMEOUT=1000;

const char* ssid="BROVAEGT";
const char* password="brovaegt123";

WebServer server(80);
HX711 scale;

long offset=0;
const float calibrationFactor=21032.35;
float currentWeight=0.0;
bool stable=false;
float lastWeight=0.0;
float lastStableWeight=0.0;
unsigned long stableTimer=0;

byte rfidFrame[RFID_MAX_FRAME_LENGTH];
size_t rfidFrameLength=0;
String lastRfidEpc="";
unsigned long lastRfidTagSeen=0;

long averageRead(int n){
  long long s=0;
  for(int i=0;i<n;i++){
    while(!scale.is_ready()) delay(1);
    s+=scale.read();
  }
  return s/n;
}

void tare(){
  offset=averageRead(50);
}

String page(){
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
body{background:#202020;color:white;font-family:Arial;text-align:center;}
h1{font-size:42px;}
#weight{font-size:80px;font-weight:bold;}
#status{font-size:30px;color:#00ff00;}
</style></head><body>
<h1>BROVÆGT</h1>
<div id='weight'>0.000 kg</div>
<div id='status'>MÅLER</div><div id='last'>Sidste: 0.000 kg</div>
<script>
setInterval(()=>{
 fetch('/weight').then(r=>r.text()).then(t=>{
 const p=t.split(';');
 document.getElementById('weight').innerHTML=p[0]+' kg';
 document.getElementById('status').innerHTML=p[1];
 document.getElementById('status').style.color=(p[1]=='STABIL')?'#00ff00':'#ffd000';
 document.getElementById('last').innerHTML='Sidste: '+p[2]+' kg';
});
},200);
</script>
</body></html>)rawliteral";
}

void handleRoot(){ server.send(200,"text/html",page()); }

void handleWeight(){
  server.send(200,"text/plain",
    String(currentWeight,3)+";"+
    (stable?"STABIL":"MÅLER")+";"+
    String(lastStableWeight,3));
}

void resetRfidFrame(){
  rfidFrameLength=0;
}

bool rfidFrameChecksumValid(){
  byte checksum=0;
  for(size_t i=1;i<rfidFrameLength-2;i++){
    checksum+=rfidFrame[i];
  }
  return checksum==rfidFrame[rfidFrameLength-2];
}

void processRfidTagFrame(){
  int epcBytes=(((rfidFrame[6]<<8)|rfidFrame[7])>>11&0x1F)*2;
  if(epcBytes<=0 || 8+epcBytes>rfidFrameLength-4) return;

  String epc="";
  for(int i=0;i<epcBytes;i++){
    if(rfidFrame[8+i]<0x10) epc+="0";
    epc+=String(rfidFrame[8+i],HEX);
  }
  epc.toUpperCase();

  if(epc!=lastRfidEpc){
    Serial.print("RFID TAG: ");
    Serial.println(epc);
    lastRfidEpc=epc;
  }
  lastRfidTagSeen=millis();
}

void handleRfidInput(){
  while(Serial2.available()>0){
    byte rfidByte=Serial2.read();

    if(rfidFrameLength==0){
      if(rfidByte==0xAA){
        rfidFrame[rfidFrameLength++]=rfidByte;
      }
      continue;
    }

    if(rfidByte==0xAA){
      rfidFrame[0]=rfidByte;
      rfidFrameLength=1;
      continue;
    }

    if(rfidFrameLength>=RFID_MAX_FRAME_LENGTH){
      resetRfidFrame();
      continue;
    }

    rfidFrame[rfidFrameLength++]=rfidByte;

    if(rfidFrameLength>=5){
      size_t expectedLength=((size_t)rfidFrame[3]<<8|rfidFrame[4])+7;
      if(expectedLength>RFID_MAX_FRAME_LENGTH){
        resetRfidFrame();
      } else if(rfidFrameLength==expectedLength){
        if(rfidFrame[rfidFrameLength-1]==0xDD &&
           rfidFrame[1]==0x02 &&
           rfidFrame[2]==0x22 &&
           rfidFrameChecksumValid()){
          processRfidTagFrame();
        }
        resetRfidFrame();
      } else if(rfidFrameLength>expectedLength){
        resetRfidFrame();
      }
    }
  }
}

void updateRfidTagPresence(){
  if(lastRfidEpc.length()>0 && millis()-lastRfidTagSeen>RFID_TAG_TIMEOUT){
    lastRfidEpc="";
  }
}

void setup(){
  Serial.begin(115200);
  scale.begin(DOUT,SCK);
  delay(1000);
  tare();

  Serial2.begin(115200,SERIAL_8N1,RFID_RX_PIN,RFID_TX_PIN);
  Serial2.write(READ_MULTI_CMD,sizeof(READ_MULTI_CMD));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  server.on("/",handleRoot);
  server.on("/weight",handleWeight);
  server.begin();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop(){
  handleRfidInput();
  updateRfidTagPresence();

  long raw=averageRead(20);
  currentWeight=(offset-raw)/calibrationFactor;

  if (fabs(currentWeight-lastWeight)<0.005){
    if(stableTimer==0) stableTimer=millis();
    if(millis()-stableTimer>2000){
      stable=true;
      lastStableWeight=currentWeight;
    }
  } else {
    stable=false;
    stableTimer=0;
  }

  lastWeight=currentWeight;
  server.handleClient();
}
