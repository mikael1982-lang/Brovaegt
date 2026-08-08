#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "HX711.h"

#define DOUT 4
#define SCK  5

const int RFID_RX_PIN = 16;
const int RFID_TX_PIN = 17;
const byte SINGLE_POLL_CMD[7] = {0xAA, 0x00, 0x22, 0x00, 0x00, 0x22, 0xDD};
const byte STOP_MULTI_CMD[7] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
const size_t RFID_MAX_FRAME_LENGTH = 64;
const unsigned long RFID_POLL_INTERVAL = 300;
const float VEHICLE_PRESENT_WEIGHT = 0.100;
const float VEHICLE_LEFT_WEIGHT = 0.050;
const unsigned long VEHICLE_LEFT_DELAY = 1500;

const char* ssid = "BROVAEGT";
const char* password = "brovaegt123";

WebServer server(80);
HX711 scale;
Preferences preferences;

long offset = 0;
const float calibrationFactor = 21032.35;
float currentWeight = 0.0;
bool stable = false;
float lastWeight = 0.0;
float lastStableWeight = 0.0;
unsigned long stableTimer = 0;

byte rfidFrame[RFID_MAX_FRAME_LENGTH];
size_t rfidFrameLength = 0;
String lastRfidEpc = "";
bool rfidStopSent = false;
bool rfidSearching = true;
unsigned long lastRfidPoll = 0;
bool vehicleWasOnScale = false;
unsigned long vehicleLeftTimer = 0;

String identifiedEpc = "";
String identifiedVehicleName = "";
float identifiedVehicleTare = 0.0;
bool identifiedVehicleKnown = false;

const int MAX_VEHICLES = 20;
struct VehicleEntry { String epc; String name; float tare; };
VehicleEntry vehicles[MAX_VEHICLES];
int vehicleCount = 0;

long averageRead(int n) {
  long long s = 0;
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) delay(1);
    s += scale.read();
  }
  return s / n;
}

void tare() { offset = averageRead(50); }

String cleanText(String value) {
  value.trim();
  value.replace(";", " ");
  value.replace("\n", " ");
  value.replace("\r", " ");
  value.replace("\t", " ");
  return value;
}

String jsonEscape(const String& value) {
  String out = "";
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

void loadVehicles() {
  preferences.begin("vehicles", false);
  vehicleCount = preferences.getUInt("count", 0);
  if (vehicleCount > MAX_VEHICLES) vehicleCount = MAX_VEHICLES;
  for (int i = 0; i < vehicleCount; i++) {
    String eKey = "e" + String(i), nKey = "n" + String(i), tKey = "t" + String(i);
    vehicles[i].epc = preferences.getString(eKey.c_str(), "");
    vehicles[i].name = preferences.getString(nKey.c_str(), "");
    vehicles[i].tare = preferences.getFloat(tKey.c_str(), 0.0);
  }
}

void saveVehicles() {
  preferences.putUInt("count", vehicleCount);
  for (int i = 0; i < MAX_VEHICLES; i++) {
    String eKey = "e" + String(i), nKey = "n" + String(i), tKey = "t" + String(i);
    if (i < vehicleCount) {
      preferences.putString(eKey.c_str(), vehicles[i].epc);
      preferences.putString(nKey.c_str(), vehicles[i].name);
      preferences.putFloat(tKey.c_str(), vehicles[i].tare);
    } else {
      preferences.remove(eKey.c_str()); preferences.remove(nKey.c_str()); preferences.remove(tKey.c_str());
    }
  }
}

int findVehicleIndex(const String& epc) {
  for (int i = 0; i < vehicleCount; i++) if (vehicles[i].epc == epc) return i;
  return -1;
}

bool lookupVehicle(const String& epc, String& name, float& tareWeight) {
  int index = findVehicleIndex(epc);
  if (index < 0) { name = ""; tareWeight = 0.0; return false; }
  name = vehicles[index].name; tareWeight = vehicles[index].tare; return true;
}

String page() {
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
body{background:#202020;color:white;font-family:Arial;text-align:center;margin:0;padding:18px}.wrap{max-width:760px;margin:auto}h1{font-size:42px;margin:10px 0 18px}#weight{font-size:80px;font-weight:bold}#status{font-size:30px;color:#00ff00}#last{font-size:20px;margin-top:6px}.info,.admin{margin-top:24px;padding:18px;border:1px solid #555;border-radius:14px;background:#292929}.admin{text-align:left}.admin h2{text-align:center}#vehicle{font-size:32px;font-weight:bold}#epc,#tare,#net{font-size:22px;margin-top:9px}#epc,.vehicleEpc{font-family:monospace;word-break:break-all}label{display:block;margin-top:12px;font-size:17px}input{width:100%;box-sizing:border-box;padding:12px;margin-top:5px;border-radius:8px;border:1px solid #777;font-size:18px}button{padding:12px 16px;margin-top:12px;border:0;border-radius:8px;font-size:17px;font-weight:bold}.primary{background:#45a049;color:white;width:100%}.secondary{background:#555;color:white}.danger{background:#a43b3b;color:white}.vehicleRow{border-top:1px solid #555;padding:13px 0}.vehicleName{font-size:19px;font-weight:bold}.vehicleEpc{font-size:13px;color:#ccc;margin-top:4px}.rowButtons button{margin-right:8px}#message{text-align:center;min-height:22px;margin-top:10px;color:#ffd000}
</style></head><body><div class='wrap'>
<h1>BROVÆGT</h1><div id='weight'>0.000 kg</div><div id='status'>MÅLER</div><div id='last'>Sidste: 0.000 kg</div>
<div class='info'><div id='vehicle'>Køretøj: --</div><div id='epc'>RFID: --</div><div id='tare'>Tomvægt: --</div><div id='net'>Nettovægt: --</div></div>
<div class='admin'><h2>Køretøjsregister</h2>
<label>RFID / EPC</label><input id='regEpc' type='text' placeholder='Scan et tag eller indsæt EPC'><button id='useCurrent' class='secondary' type='button'>Brug aktuelt scannet RFID</button>
<label>Køretøjsnavn</label><input id='regName' type='text' placeholder='F.eks. Scania 770'>
<label>Tomvægt (kg)</label><input id='regTare' type='text' inputmode='decimal' placeholder='0,500 eller 0.500'>
<button id='saveVehicle' class='primary' type='button'>Gem køretøj</button><div id='message'></div><div id='vehicleList'></div></div>
<script>
var currentEpc=''; var currentName=''; var currentTare='';
function byId(id){return document.getElementById(id)}
function trimText(s){return String(s).replace(/^\s+|\s+$/g,'')}
function setMessage(msg){byId('message').innerHTML=msg}
function encodeForm(v){return encodeURIComponent(v).replace(/%20/g,'+')}
function request(method,url,body,ok,fail){
 var x=new XMLHttpRequest(); x.open(method,url,true);
 if(method=='POST')x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
 x.onreadystatechange=function(){if(x.readyState==4){if(x.status>=200&&x.status<300){if(ok)ok(x.responseText)}else{if(fail)fail(x.responseText||('Fejl '+x.status))}}};
 x.send(body||null);
}
function refreshWeight(){request('GET','/weight',null,function(t){var p=t.split(';');byId('weight').innerHTML=p[0]+' kg';byId('status').innerHTML=p[1];byId('status').style.color=(p[1]=='STABIL')?'#00ff00':'#ffd000';byId('last').innerHTML='Sidste: '+p[2]+' kg';currentEpc=(p[3]&&p[3]!='-')?p[3]:'';currentName=(p[4]&&p[4]!='-'&&p[4]!='UKENDT')?p[4]:'';currentTare=(p[5]&&p[5]!='-')?p[5]:'';byId('epc').innerHTML='RFID: '+(currentEpc||'--');byId('vehicle').innerHTML='Køretøj: '+((p[4]&&p[4]!='-')?p[4]:'--');byId('tare').innerHTML='Tomvægt: '+(currentTare?currentTare+' kg':'--');byId('net').innerHTML='Nettovægt: '+((p[6]&&p[6]!='-')?p[6]+' kg':'--')})}
function editVehicle(epc,name,tare){byId('regEpc').value=epc;byId('regName').value=name;byId('regTare').value=Number(tare).toFixed(3);setMessage('Køretøj klar til redigering.')}
function deleteVehicle(epc,name){if(!confirm('Slet '+name+'?'))return;request('POST','/vehicle/delete','epc='+encodeForm(epc),function(){setMessage('Køretøj slettet.');refreshVehicles();refreshWeight()},function(e){setMessage(e)})}
function refreshVehicles(){request('GET','/vehicles',null,function(txt){var list;try{list=JSON.parse(txt)}catch(e){setMessage('Kunne ikke læse køretøjslisten.');return}var box=byId('vehicleList');box.innerHTML='';if(!list.length){box.innerHTML='<div class="vehicleRow">Ingen køretøjer gemt endnu.</div>';return}for(var i=0;i<list.length;i++){var v=list[i];var row=document.createElement('div');row.className='vehicleRow';var n=document.createElement('div');n.className='vehicleName';n.appendChild(document.createTextNode(v.name));row.appendChild(n);var e=document.createElement('div');e.className='vehicleEpc';e.appendChild(document.createTextNode(v.epc));row.appendChild(e);var t=document.createElement('div');t.appendChild(document.createTextNode('Tomvægt: '+Number(v.tare).toFixed(3)+' kg'));row.appendChild(t);var eb=document.createElement('button');eb.className='secondary';eb.type='button';eb.innerHTML='Redigér';eb.setAttribute('data-i',i);eb.onclick=function(){var z=list[parseInt(this.getAttribute('data-i'),10)];editVehicle(z.epc,z.name,z.tare)};row.appendChild(eb);var db=document.createElement('button');db.className='danger';db.type='button';db.innerHTML='Slet';db.setAttribute('data-i',i);db.onclick=function(){var z=list[parseInt(this.getAttribute('data-i'),10)];deleteVehicle(z.epc,z.name)};row.appendChild(db);box.appendChild(row)}})}
function saveVehicle(){var epc=trimText(byId('regEpc').value).toUpperCase();var name=trimText(byId('regName').value);var tare=trimText(byId('regTare').value).replace(',','.');if(!epc||!name||tare===''){setMessage('Udfyld RFID, navn og tomvægt.');return}if(isNaN(Number(tare))||Number(tare)<0){setMessage('Tomvægt er ikke et gyldigt tal.');return}var body='epc='+encodeForm(epc)+'&name='+encodeForm(name)+'&tare='+encodeForm(tare);setMessage('Gemmer...');request('POST','/vehicle/save',body,function(){setMessage('Køretøj gemt.');byId('regEpc').value='';byId('regName').value='';byId('regTare').value='';refreshVehicles();refreshWeight()},function(e){setMessage(e)})}
byId('useCurrent').onclick=function(){if(!currentEpc){setMessage('Intet RFID er scannet endnu.');return}byId('regEpc').value=currentEpc;byId('regName').value=currentName;byId('regTare').value=currentTare;setMessage('Aktuelt RFID er lagt i formularen.')};byId('saveVehicle').onclick=saveVehicle;
setInterval(refreshWeight,500);refreshWeight();refreshVehicles();
</script></div></body></html>)rawliteral";
}

void handleRoot(){server.send(200,"text/html",page());}
void handleWeight(){String tareText="-",netText="-",vehicleText="UKENDT";if(identifiedVehicleKnown){tareText=String(identifiedVehicleTare,3);netText=String(currentWeight-identifiedVehicleTare,3);vehicleText=identifiedVehicleName;}server.send(200,"text/plain",String(currentWeight,3)+";"+(stable?"STABIL":"MÅLER")+";"+String(lastStableWeight,3)+";"+(identifiedEpc.length()>0?identifiedEpc:"-")+";"+(identifiedEpc.length()>0?vehicleText:"-")+";"+tareText+";"+netText);}
void handleVehicles(){String json="[";for(int i=0;i<vehicleCount;i++){if(i>0)json+=",";json+="{\"epc\":\""+jsonEscape(vehicles[i].epc)+"\",\"name\":\""+jsonEscape(vehicles[i].name)+"\",\"tare\":"+String(vehicles[i].tare,3)+"}";}json+="]";server.send(200,"application/json",json);}

void handleVehicleSave(){
  if(!server.hasArg("epc")||!server.hasArg("name")||!server.hasArg("tare")){server.send(400,"text/plain","Mangler RFID, navn eller tomvægt.");return;}
  String epc=cleanText(server.arg("epc"));epc.toUpperCase();String name=cleanText(server.arg("name"));String tareText=cleanText(server.arg("tare"));tareText.replace(",",".");
  if(epc.length()==0||name.length()==0||tareText.length()==0){server.send(400,"text/plain","Udfyld RFID, navn og tomvægt.");return;}
  float tareWeight=tareText.toFloat();if(tareWeight<0.0){server.send(400,"text/plain","Tomvægt må ikke være negativ.");return;}
  int index=findVehicleIndex(epc);if(index<0){if(vehicleCount>=MAX_VEHICLES){server.send(400,"text/plain","Registeret er fuldt (maks. 20 køretøjer).");return;}index=vehicleCount++;}
  vehicles[index].epc=epc;vehicles[index].name=name;vehicles[index].tare=tareWeight;saveVehicles();
  if(identifiedEpc==epc){identifiedVehicleName=name;identifiedVehicleTare=tareWeight;identifiedVehicleKnown=true;}
  Serial.print("VEHICLE SAVED: ");Serial.print(epc);Serial.print(" / ");Serial.print(name);Serial.print(" / ");Serial.println(tareWeight,3);server.send(200,"text/plain","OK");
}

void handleVehicleDelete(){if(!server.hasArg("epc")){server.send(400,"text/plain","Mangler RFID.");return;}String epc=cleanText(server.arg("epc"));epc.toUpperCase();int index=findVehicleIndex(epc);if(index<0){server.send(404,"text/plain","Køretøjet findes ikke.");return;}for(int i=index;i<vehicleCount-1;i++)vehicles[i]=vehicles[i+1];vehicleCount--;saveVehicles();if(identifiedEpc==epc){identifiedVehicleName="UKENDT";identifiedVehicleTare=0.0;identifiedVehicleKnown=false;}Serial.print("VEHICLE DELETED: ");Serial.println(epc);server.send(200,"text/plain","OK");}

void resetRfidFrame(){rfidFrameLength=0;}
bool rfidFrameChecksumValid(){byte checksum=0;for(size_t i=1;i<rfidFrameLength-2;i++)checksum+=rfidFrame[i];return checksum==rfidFrame[rfidFrameLength-2];}
void startRfidSearch(){rfidSearching=true;rfidStopSent=false;lastRfidEpc="";lastRfidPoll=0;resetRfidFrame();}
void handleRfidPolling(){if(!rfidSearching)return;if(lastRfidPoll==0||millis()-lastRfidPoll>=RFID_POLL_INTERVAL){Serial2.write(SINGLE_POLL_CMD,sizeof(SINGLE_POLL_CMD));lastRfidPoll=millis();}}

void processRfidTagFrame(){
  int epcBytes=(((rfidFrame[6]<<8)|rfidFrame[7])>>11&0x1F)*2;if(epcBytes<=0||8+epcBytes>rfidFrameLength-4)return;
  String epc="";for(int i=0;i<epcBytes;i++){if(rfidFrame[8+i]<0x10)epc+="0";epc+=String(rfidFrame[8+i],HEX);}epc.toUpperCase();
  if(epc!=lastRfidEpc){Serial.print("[");Serial.print(millis());Serial.print(" ms] RFID TAG: ");Serial.println(epc);lastRfidEpc=epc;}
  identifiedEpc=epc;identifiedVehicleKnown=lookupVehicle(epc,identifiedVehicleName,identifiedVehicleTare);if(!identifiedVehicleKnown){identifiedVehicleName="UKENDT";identifiedVehicleTare=0.0;}rfidSearching=false;
  if(!rfidStopSent){Serial2.write(STOP_MULTI_CMD,sizeof(STOP_MULTI_CMD));Serial.print("[");Serial.print(millis());Serial.println(" ms] STOP POLL SENT");rfidStopSent=true;}
}

void handleRfidInput(){
  while(Serial2.available()>0){byte rfidByte=Serial2.read();if(rfidFrameLength==0){if(rfidByte==0xAA)rfidFrame[rfidFrameLength++]=rfidByte;continue;}if(rfidByte==0xAA){rfidFrame[0]=rfidByte;rfidFrameLength=1;continue;}if(rfidFrameLength>=RFID_MAX_FRAME_LENGTH){resetRfidFrame();continue;}rfidFrame[rfidFrameLength++]=rfidByte;if(rfidFrameLength>=5){size_t expectedLength=((size_t)rfidFrame[3]<<8|rfidFrame[4])+7;if(expectedLength>RFID_MAX_FRAME_LENGTH)resetRfidFrame();else if(rfidFrameLength==expectedLength){if(rfidFrame[rfidFrameLength-1]==0xDD&&rfidFrame[1]==0x02&&rfidFrame[2]==0x22&&rfidFrameChecksumValid())processRfidTagFrame();resetRfidFrame();}else if(rfidFrameLength>expectedLength)resetRfidFrame();}}
}

void updateVehicleCycle(){if(identifiedEpc.length()==0)return;if(currentWeight>VEHICLE_PRESENT_WEIGHT){vehicleWasOnScale=true;vehicleLeftTimer=0;}if(vehicleWasOnScale&&currentWeight<VEHICLE_LEFT_WEIGHT){if(vehicleLeftTimer==0)vehicleLeftTimer=millis();if(millis()-vehicleLeftTimer>=VEHICLE_LEFT_DELAY){identifiedEpc="";identifiedVehicleName="";identifiedVehicleTare=0.0;identifiedVehicleKnown=false;vehicleWasOnScale=false;vehicleLeftTimer=0;startRfidSearch();Serial.println("RFID READY FOR NEXT VEHICLE");}}else if(currentWeight>=VEHICLE_LEFT_WEIGHT)vehicleLeftTimer=0;}

void setup(){Serial.begin(115200);scale.begin(DOUT,SCK);delay(1000);tare();loadVehicles();Serial.print("VEHICLES LOADED: ");Serial.println(vehicleCount);Serial2.begin(115200,SERIAL_8N1,RFID_RX_PIN,RFID_TX_PIN);startRfidSearch();WiFi.mode(WIFI_AP);WiFi.softAP(ssid,password);server.on("/",handleRoot);server.on("/weight",handleWeight);server.on("/vehicles",HTTP_GET,handleVehicles);server.on("/vehicle/save",HTTP_POST,handleVehicleSave);server.on("/vehicle/delete",HTTP_POST,handleVehicleDelete);server.begin();Serial.print("AP IP: ");Serial.println(WiFi.softAPIP());}

void loop(){handleRfidInput();handleRfidPolling();long raw=averageRead(20);currentWeight=(offset-raw)/calibrationFactor;if(fabs(currentWeight-lastWeight)<0.005){if(stableTimer==0)stableTimer=millis();if(millis()-stableTimer>2000){stable=true;lastStableWeight=currentWeight;}}else{stable=false;stableTimer=0;}updateVehicleCycle();lastWeight=currentWeight;server.handleClient();}
