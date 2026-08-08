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

// RFID starts when at least 0.500 kg is physically on the bridge.
const float RFID_TRIGGER_WEIGHT = 0.500;
const float VEHICLE_LEFT_WEIGHT = 0.050;
const unsigned long VEHICLE_LEFT_DELAY = 1500;

// 1:14 geometric volume scaling for payload material only.
const float LOAD_SCALE_FACTOR = 2744.0;

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
bool rfidSearching = false;
unsigned long lastRfidPoll = 0;
bool vehicleWasOnScale = false;
unsigned long vehicleLeftTimer = 0;

String identifiedEpc = "";
String identifiedVehicleName = "";
String identifiedVehicleOwner = "";
float identifiedModelTare = 0.0;
float identifiedRealTare = 0.0;
float identifiedMaxGross = 0.0;
bool identifiedVehicleKnown = false;

const int MAX_VEHICLES = 20;
struct VehicleEntry {
  String epc;
  String name;
  String owner;
  float modelTare;
  float realTare;
  float maxGross;
};
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

void tareScale() { offset = averageRead(50); }

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
    String eKey = "e" + String(i);
    String nKey = "n" + String(i);
    String oKey = "o" + String(i);
    String tKey = "t" + String(i);   // old v1.5.x tare key: preserved as model tare
    String rKey = "r" + String(i);
    String gKey = "g" + String(i);

    vehicles[i].epc = preferences.getString(eKey.c_str(), "");
    vehicles[i].name = preferences.getString(nKey.c_str(), "");
    vehicles[i].owner = preferences.getString(oKey.c_str(), "");
    vehicles[i].modelTare = preferences.getFloat(tKey.c_str(), 0.0);
    vehicles[i].realTare = preferences.getFloat(rKey.c_str(), 0.0);
    vehicles[i].maxGross = preferences.getFloat(gKey.c_str(), 0.0);
  }
}

void saveVehicles() {
  preferences.putUInt("count", vehicleCount);
  for (int i = 0; i < MAX_VEHICLES; i++) {
    String eKey = "e" + String(i);
    String nKey = "n" + String(i);
    String oKey = "o" + String(i);
    String tKey = "t" + String(i);
    String rKey = "r" + String(i);
    String gKey = "g" + String(i);

    if (i < vehicleCount) {
      preferences.putString(eKey.c_str(), vehicles[i].epc);
      preferences.putString(nKey.c_str(), vehicles[i].name);
      preferences.putString(oKey.c_str(), vehicles[i].owner);
      preferences.putFloat(tKey.c_str(), vehicles[i].modelTare);
      preferences.putFloat(rKey.c_str(), vehicles[i].realTare);
      preferences.putFloat(gKey.c_str(), vehicles[i].maxGross);
    } else {
      preferences.remove(eKey.c_str());
      preferences.remove(nKey.c_str());
      preferences.remove(oKey.c_str());
      preferences.remove(tKey.c_str());
      preferences.remove(rKey.c_str());
      preferences.remove(gKey.c_str());
    }
  }
}

int findVehicleIndex(const String& epc) {
  for (int i = 0; i < vehicleCount; i++) {
    if (vehicles[i].epc == epc) return i;
  }
  return -1;
}

bool lookupVehicle(const String& epc) {
  int index = findVehicleIndex(epc);
  if (index < 0) return false;
  identifiedVehicleName = vehicles[index].name;
  identifiedVehicleOwner = vehicles[index].owner;
  identifiedModelTare = vehicles[index].modelTare;
  identifiedRealTare = vehicles[index].realTare;
  identifiedMaxGross = vehicles[index].maxGross;
  return true;
}

void clearIdentifiedVehicle() {
  identifiedEpc = "";
  identifiedVehicleName = "";
  identifiedVehicleOwner = "";
  identifiedModelTare = 0.0;
  identifiedRealTare = 0.0;
  identifiedMaxGross = 0.0;
  identifiedVehicleKnown = false;
}

float calculatedRealLoad() {
  if (!identifiedVehicleKnown) return 0.0;
  float modelLoad = currentWeight - identifiedModelTare;
  if (modelLoad < 0.0) modelLoad = 0.0;
  return modelLoad * LOAD_SCALE_FACTOR;
}

float calculatedRealGross() {
  return identifiedRealTare + calculatedRealLoad();
}

String mainPage() {
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>
<meta name='apple-mobile-web-app-capable' content='yes'>
<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent'>
<meta name='apple-mobile-web-app-title' content='Brovægt'>
<title>Brovægt</title>
<style>
html,body{margin:0;padding:0;background:#151515;color:#fff;font-family:Arial,sans-serif;min-height:100%;}
.wrap{max-width:900px;margin:auto;padding:20px;text-align:center;box-sizing:border-box;}
h1{font-size:42px;margin:5px 0 12px;letter-spacing:2px;}
#measureStatus{font-size:26px;font-weight:bold;color:#ffd000;margin-bottom:20px;}
.vehicleCard{background:#242424;border:1px solid #555;border-radius:16px;padding:20px;margin-bottom:18px;}
#vehicle{font-size:38px;font-weight:bold;}
#owner{font-size:22px;margin-top:8px;color:#ccc;}
.grid{display:table;width:100%;border-spacing:12px;table-layout:fixed;}
.cell{display:table-cell;background:#242424;border:1px solid #555;border-radius:16px;padding:18px 8px;vertical-align:middle;}
.label{font-size:18px;color:#bbb;margin-bottom:8px;text-transform:uppercase;}
.value{font-size:40px;font-weight:bold;white-space:nowrap;}
#result{font-size:42px;font-weight:bold;border-radius:16px;padding:18px;margin:12px 0 18px;background:#444;}
.ok{background:#245b2c!important;}.warn{background:#9a6a00!important;}.over{background:#9a2424!important;}.unknown{background:#555!important;}
.small{font-size:20px;margin-top:6px;}
.adminLink{display:inline-block;margin-top:12px;padding:10px 18px;background:#333;color:#bbb;text-decoration:none;border-radius:8px;font-size:16px;}
@media(max-width:650px){.grid{display:block;border-spacing:0}.cell{display:block;margin-bottom:12px}.value{font-size:34px}#vehicle{font-size:32px}#result{font-size:34px}}
</style></head><body><div class='wrap'>
<h1>BROVÆGT</h1>
<div id='measureStatus'>VENTER</div>
<div class='vehicleCard'><div id='vehicle'>Intet køretøj</div><div id='owner'>Ejer: --</div></div>
<div class='grid'>
  <div class='cell'><div class='label'>Egenvægt</div><div id='tare' class='value'>--</div></div>
  <div class='cell'><div class='label'>Last</div><div id='load' class='value'>--</div></div>
  <div class='cell'><div class='label'>Totalvægt</div><div id='gross' class='value'>--</div></div>
</div>
<div id='result' class='unknown'>VENTER PÅ KØRETØJ</div>
<div id='overBy' class='small'></div>
<a class='adminLink' href='/vehicles'>Køretøjsregister</a>
<script>
function byId(id){return document.getElementById(id)}
function formatKg(v){var n=Math.round(Number(v));var s=String(Math.abs(n));var out='';while(s.length>3){out='.'+s.substr(s.length-3)+out;s=s.substr(0,s.length-3)}out=s+out;if(n<0)out='-'+out;return out+' kg'}
function request(){var x=new XMLHttpRequest();x.open('GET','/status',true);x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){var p=x.responseText.split(';');var known=p[0]=='1';var onScale=p[1]=='1';var stable=p[2]=='1';byId('measureStatus').innerHTML=onScale?(stable?'STABIL':'MÅLER'):'VENTER';byId('measureStatus').style.color=stable?'#00ff00':'#ffd000';if(!onScale){byId('vehicle').innerHTML='Intet køretøj';byId('owner').innerHTML='Ejer: --';byId('tare').innerHTML='--';byId('load').innerHTML='--';byId('gross').innerHTML='--';byId('result').className='unknown';byId('result').innerHTML='VENTER PÅ KØRETØJ';byId('overBy').innerHTML='';return}if(!known){byId('vehicle').innerHTML='UKENDT KØRETØJ';byId('owner').innerHTML='Ejer: --';byId('tare').innerHTML='--';byId('load').innerHTML='--';byId('gross').innerHTML='--';byId('result').className='warn';byId('result').innerHTML='KØRETØJ IKKE REGISTRERET';byId('overBy').innerHTML='';return}byId('vehicle').innerHTML=p[3]||'Køretøj';byId('owner').innerHTML='Ejer: '+(p[4]||'--');byId('tare').innerHTML=formatKg(p[5]);byId('load').innerHTML=formatKg(p[6]);byId('gross').innerHTML=formatKg(p[7]);var max=Number(p[8]);var over=p[9]=='1';if(!stable){byId('result').className='unknown';byId('result').innerHTML='MÅLER';byId('overBy').innerHTML='';}else if(max<=0){byId('result').className='unknown';byId('result').innerHTML='INGEN VÆGTGRÆNSE';byId('overBy').innerHTML='';}else if(over){byId('result').className='over';byId('result').innerHTML='OVERLÆS';byId('overBy').innerHTML='Over med '+formatKg(Number(p[7])-max)+' · Tilladt '+formatKg(max);}else{byId('result').className='ok';byId('result').innerHTML='OK';byId('overBy').innerHTML='Tilladt totalvægt: '+formatKg(max);}}};x.send(null)}
setInterval(request,500);request();
</script></div></body></html>)rawliteral";
}

String vehiclesPage() {
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<meta name='apple-mobile-web-app-capable' content='yes'>
<title>Køretøjsregister</title>
<style>
body{background:#202020;color:white;font-family:Arial;margin:0;padding:18px}.wrap{max-width:760px;margin:auto}h1{text-align:center}.back{display:inline-block;padding:10px 14px;background:#444;color:white;text-decoration:none;border-radius:8px;margin-bottom:12px}.box{padding:18px;border:1px solid #555;border-radius:14px;background:#292929;margin-bottom:18px}label{display:block;margin-top:12px;font-size:17px}input{width:100%;box-sizing:border-box;padding:12px;margin-top:5px;border-radius:8px;border:1px solid #777;font-size:18px}button{padding:12px 16px;margin-top:12px;border:0;border-radius:8px;font-size:17px;font-weight:bold}.primary{background:#45a049;color:white;width:100%}.secondary{background:#555;color:white}.danger{background:#a43b3b;color:white}.vehicleRow{border-top:1px solid #555;padding:15px 0}.vehicleName{font-size:21px;font-weight:bold}.vehicleMeta{font-size:15px;color:#ccc;margin-top:5px}.vehicleEpc{font-family:monospace;word-break:break-all;font-size:12px;color:#aaa;margin-top:5px}#message{text-align:center;min-height:22px;margin-top:10px;color:#ffd000}
</style></head><body><div class='wrap'>
<a class='back' href='/'>← Brovægt</a><h1>Køretøjsregister</h1>
<div class='box'>
<label>RFID / EPC</label><input id='regEpc' type='text' placeholder='Scan et tag eller indsæt EPC'><button id='useCurrent' class='secondary' type='button'>Brug aktuelt scannet RFID</button>
<label>Køretøjsnavn</label><input id='regName' type='text' placeholder='F.eks. Kroghejs'>
<label>Ejer</label><input id='regOwner' type='text' placeholder='F.eks. Mikael'>
<label>Model egenvægt (kg)</label><input id='regModelTare' type='text' inputmode='decimal' placeholder='F.eks. 8,000'>
<label>Virkelig egenvægt (kg)</label><input id='regRealTare' type='text' inputmode='numeric' placeholder='F.eks. 15500'>
<label>Maks. tilladt totalvægt (kg)</label><input id='regMaxGross' type='text' inputmode='numeric' placeholder='F.eks. 26000'>
<button id='saveVehicle' class='primary' type='button'>Gem køretøj</button><div id='message'></div>
</div>
<div class='box'><h2>Registrerede køretøjer</h2><div id='vehicleList'></div></div>
<script>
var currentEpc='';
function byId(id){return document.getElementById(id)}
function trimText(s){return String(s).replace(/^\s+|\s+$/g,'')}
function setMessage(msg){byId('message').innerHTML=msg}
function enc(v){return encodeURIComponent(v).replace(/%20/g,'+')}
function ajax(method,url,body,ok,fail){var x=new XMLHttpRequest();x.open(method,url,true);if(method=='POST')x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');x.onreadystatechange=function(){if(x.readyState==4){if(x.status>=200&&x.status<300){if(ok)ok(x.responseText)}else{if(fail)fail(x.responseText||('Fejl '+x.status))}}};x.send(body||null)}
function getCurrent(){ajax('GET','/current-rfid',null,function(t){currentEpc=trimText(t)})}
function editVehicle(v){byId('regEpc').value=v.epc;byId('regName').value=v.name;byId('regOwner').value=v.owner;byId('regModelTare').value=Number(v.modelTare).toFixed(3);byId('regRealTare').value=Math.round(Number(v.realTare));byId('regMaxGross').value=Math.round(Number(v.maxGross));setMessage('Køretøj klar til redigering.');window.scrollTo(0,0)}
function deleteVehicle(epc,name){if(!confirm('Slet '+name+'?'))return;ajax('POST','/vehicle/delete','epc='+enc(epc),function(){setMessage('Køretøj slettet.');refreshVehicles()},function(e){setMessage(e)})}
function refreshVehicles(){ajax('GET','/vehicles.json',null,function(txt){var list;try{list=JSON.parse(txt)}catch(e){setMessage('Kunne ikke læse køretøjslisten.');return}var box=byId('vehicleList');box.innerHTML='';if(!list.length){box.innerHTML='Ingen køretøjer gemt endnu.';return}for(var i=0;i<list.length;i++){(function(v){var row=document.createElement('div');row.className='vehicleRow';var n=document.createElement('div');n.className='vehicleName';n.appendChild(document.createTextNode(v.name));row.appendChild(n);var m=document.createElement('div');m.className='vehicleMeta';m.appendChild(document.createTextNode('Ejer: '+(v.owner||'--')+' · Model egenvægt: '+Number(v.modelTare).toFixed(3)+' kg · Virkelig egenvægt: '+Math.round(Number(v.realTare))+' kg · Maks: '+Math.round(Number(v.maxGross))+' kg'));row.appendChild(m);var e=document.createElement('div');e.className='vehicleEpc';e.appendChild(document.createTextNode(v.epc));row.appendChild(e);var eb=document.createElement('button');eb.className='secondary';eb.type='button';eb.innerHTML='Redigér';eb.onclick=function(){editVehicle(v)};row.appendChild(eb);var db=document.createElement('button');db.className='danger';db.type='button';db.innerHTML='Slet';db.onclick=function(){deleteVehicle(v.epc,v.name)};row.appendChild(db);box.appendChild(row)})(list[i])}})}
function num(id){return trimText(byId(id).value).replace(',','.')}
function saveVehicle(){var epc=trimText(byId('regEpc').value).toUpperCase();var name=trimText(byId('regName').value);var owner=trimText(byId('regOwner').value);var mt=num('regModelTare');var rt=num('regRealTare');var mg=num('regMaxGross');if(!epc||!name||!owner||mt===''||rt===''||mg===''){setMessage('Udfyld alle felter.');return}if(isNaN(Number(mt))||isNaN(Number(rt))||isNaN(Number(mg))||Number(mt)<0||Number(rt)<0||Number(mg)<0){setMessage('Vægte skal være gyldige positive tal.');return}var body='epc='+enc(epc)+'&name='+enc(name)+'&owner='+enc(owner)+'&modelTare='+enc(mt)+'&realTare='+enc(rt)+'&maxGross='+enc(mg);setMessage('Gemmer...');ajax('POST','/vehicle/save',body,function(){setMessage('Køretøj gemt.');byId('regEpc').value='';byId('regName').value='';byId('regOwner').value='';byId('regModelTare').value='';byId('regRealTare').value='';byId('regMaxGross').value='';refreshVehicles()},function(e){setMessage(e)})}
byId('useCurrent').onclick=function(){getCurrent();setTimeout(function(){if(!currentEpc){setMessage('Intet RFID er scannet endnu.');return}byId('regEpc').value=currentEpc;setMessage('Aktuelt RFID er lagt i formularen.')},200)};byId('saveVehicle').onclick=saveVehicle;
getCurrent();refreshVehicles();
</script></div></body></html>)rawliteral";
}

void handleRoot() { server.send(200, "text/html", mainPage()); }
void handleVehiclesPage() { server.send(200, "text/html", vehiclesPage()); }

void handleStatus() {
  bool onScale = currentWeight >= RFID_TRIGGER_WEIGHT || vehicleWasOnScale;
  float load = calculatedRealLoad();
  float gross = calculatedRealGross();
  bool overload = identifiedVehicleKnown && stable && identifiedMaxGross > 0.0 && gross > identifiedMaxGross;

  server.send(200, "text/plain",
    String(identifiedVehicleKnown ? 1 : 0) + ";" +
    String(onScale ? 1 : 0) + ";" +
    String(stable ? 1 : 0) + ";" +
    (identifiedVehicleKnown ? identifiedVehicleName : "") + ";" +
    (identifiedVehicleKnown ? identifiedVehicleOwner : "") + ";" +
    String(identifiedVehicleKnown ? identifiedRealTare : 0.0, 0) + ";" +
    String(identifiedVehicleKnown ? load : 0.0, 0) + ";" +
    String(identifiedVehicleKnown ? gross : 0.0, 0) + ";" +
    String(identifiedVehicleKnown ? identifiedMaxGross : 0.0, 0) + ";" +
    String(overload ? 1 : 0));
}

void handleCurrentRfid() { server.send(200, "text/plain", identifiedEpc); }

void handleVehiclesJson() {
  String json = "[";
  for (int i = 0; i < vehicleCount; i++) {
    if (i > 0) json += ",";
    json += "{\"epc\":\"" + jsonEscape(vehicles[i].epc) + "\",";
    json += "\"name\":\"" + jsonEscape(vehicles[i].name) + "\",";
    json += "\"owner\":\"" + jsonEscape(vehicles[i].owner) + "\",";
    json += "\"modelTare\":" + String(vehicles[i].modelTare, 3) + ",";
    json += "\"realTare\":" + String(vehicles[i].realTare, 0) + ",";
    json += "\"maxGross\":" + String(vehicles[i].maxGross, 0) + "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleVehicleSave() {
  if (!server.hasArg("epc") || !server.hasArg("name") || !server.hasArg("owner") ||
      !server.hasArg("modelTare") || !server.hasArg("realTare") || !server.hasArg("maxGross")) {
    server.send(400, "text/plain", "Mangler et eller flere felter.");
    return;
  }

  String epc = cleanText(server.arg("epc")); epc.toUpperCase();
  String name = cleanText(server.arg("name"));
  String owner = cleanText(server.arg("owner"));
  String modelText = cleanText(server.arg("modelTare")); modelText.replace(",", ".");
  String realText = cleanText(server.arg("realTare")); realText.replace(",", ".");
  String maxText = cleanText(server.arg("maxGross")); maxText.replace(",", ".");

  if (epc.length() == 0 || name.length() == 0 || owner.length() == 0 ||
      modelText.length() == 0 || realText.length() == 0 || maxText.length() == 0) {
    server.send(400, "text/plain", "Udfyld alle felter.");
    return;
  }

  float modelTare = modelText.toFloat();
  float realTare = realText.toFloat();
  float maxGross = maxText.toFloat();
  if (modelTare < 0.0 || realTare < 0.0 || maxGross < 0.0) {
    server.send(400, "text/plain", "Vægte må ikke være negative.");
    return;
  }

  int index = findVehicleIndex(epc);
  if (index < 0) {
    if (vehicleCount >= MAX_VEHICLES) {
      server.send(400, "text/plain", "Registeret er fuldt (maks. 20 køretøjer).");
      return;
    }
    index = vehicleCount++;
  }

  vehicles[index].epc = epc;
  vehicles[index].name = name;
  vehicles[index].owner = owner;
  vehicles[index].modelTare = modelTare;
  vehicles[index].realTare = realTare;
  vehicles[index].maxGross = maxGross;
  saveVehicles();

  if (identifiedEpc == epc) {
    identifiedVehicleKnown = lookupVehicle(epc);
  }

  Serial.print("VEHICLE SAVED: ");
  Serial.print(epc); Serial.print(" / ");
  Serial.print(name); Serial.print(" / ");
  Serial.print(owner); Serial.print(" / model ");
  Serial.print(modelTare, 3); Serial.print(" kg / real ");
  Serial.print(realTare, 0); Serial.print(" kg / max ");
  Serial.println(maxGross, 0);
  server.send(200, "text/plain", "OK");
}

void handleVehicleDelete() {
  if (!server.hasArg("epc")) { server.send(400, "text/plain", "Mangler RFID."); return; }
  String epc = cleanText(server.arg("epc")); epc.toUpperCase();
  int index = findVehicleIndex(epc);
  if (index < 0) { server.send(404, "text/plain", "Køretøjet findes ikke."); return; }
  for (int i = index; i < vehicleCount - 1; i++) vehicles[i] = vehicles[i + 1];
  vehicleCount--;
  saveVehicles();
  if (identifiedEpc == epc) {
    identifiedVehicleKnown = false;
    identifiedVehicleName = "UKENDT";
    identifiedVehicleOwner = "";
    identifiedModelTare = 0.0;
    identifiedRealTare = 0.0;
    identifiedMaxGross = 0.0;
  }
  Serial.print("VEHICLE DELETED: "); Serial.println(epc);
  server.send(200, "text/plain", "OK");
}

void resetRfidFrame() { rfidFrameLength = 0; }

bool rfidFrameChecksumValid() {
  byte checksum = 0;
  for (size_t i = 1; i < rfidFrameLength - 2; i++) checksum += rfidFrame[i];
  return checksum == rfidFrame[rfidFrameLength - 2];
}

void startRfidSearch() {
  rfidSearching = true;
  rfidStopSent = false;
  lastRfidEpc = "";
  lastRfidPoll = 0;
  resetRfidFrame();
  Serial.println("RFID SEARCH STARTED - WEIGHT >= 0.500 kg");
}

void stopRfidSearchSilently() {
  rfidSearching = false;
  rfidStopSent = false;
  lastRfidPoll = 0;
  resetRfidFrame();
}

void handleRfidPolling() {
  if (!rfidSearching) return;
  if (lastRfidPoll == 0 || millis() - lastRfidPoll >= RFID_POLL_INTERVAL) {
    Serial2.write(SINGLE_POLL_CMD, sizeof(SINGLE_POLL_CMD));
    lastRfidPoll = millis();
  }
}

void processRfidTagFrame() {
  int epcBytes = (((rfidFrame[6] << 8) | rfidFrame[7]) >> 11 & 0x1F) * 2;
  if (epcBytes <= 0 || 8 + epcBytes > (int)rfidFrameLength - 4) return;

  String epc = "";
  for (int i = 0; i < epcBytes; i++) {
    if (rfidFrame[8 + i] < 0x10) epc += "0";
    epc += String(rfidFrame[8 + i], HEX);
  }
  epc.toUpperCase();

  if (epc != lastRfidEpc) {
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] RFID TAG: "); Serial.println(epc);
    lastRfidEpc = epc;
  }

  identifiedEpc = epc;
  identifiedVehicleKnown = lookupVehicle(epc);
  if (!identifiedVehicleKnown) {
    identifiedVehicleName = "UKENDT";
    identifiedVehicleOwner = "";
    identifiedModelTare = 0.0;
    identifiedRealTare = 0.0;
    identifiedMaxGross = 0.0;
  }

  rfidSearching = false;
  if (!rfidStopSent) {
    Serial2.write(STOP_MULTI_CMD, sizeof(STOP_MULTI_CMD));
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] STOP POLL SENT");
    rfidStopSent = true;
  }
}

void handleRfidInput() {
  while (Serial2.available() > 0) {
    byte rfidByte = Serial2.read();
    if (rfidFrameLength == 0) {
      if (rfidByte == 0xAA) rfidFrame[rfidFrameLength++] = rfidByte;
      continue;
    }
    if (rfidByte == 0xAA) {
      rfidFrame[0] = rfidByte;
      rfidFrameLength = 1;
      continue;
    }
    if (rfidFrameLength >= RFID_MAX_FRAME_LENGTH) { resetRfidFrame(); continue; }
    rfidFrame[rfidFrameLength++] = rfidByte;

    if (rfidFrameLength >= 5) {
      size_t expectedLength = ((size_t)rfidFrame[3] << 8 | rfidFrame[4]) + 7;
      if (expectedLength > RFID_MAX_FRAME_LENGTH) resetRfidFrame();
      else if (rfidFrameLength == expectedLength) {
        if (rfidFrame[rfidFrameLength - 1] == 0xDD && rfidFrame[1] == 0x02 &&
            rfidFrame[2] == 0x22 && rfidFrameChecksumValid()) processRfidTagFrame();
        resetRfidFrame();
      } else if (rfidFrameLength > expectedLength) resetRfidFrame();
    }
  }
}

void updateVehicleCycle() {
  if (!vehicleWasOnScale && currentWeight >= RFID_TRIGGER_WEIGHT) {
    vehicleWasOnScale = true;
    vehicleLeftTimer = 0;
    startRfidSearch();
  }

  if (vehicleWasOnScale && currentWeight < VEHICLE_LEFT_WEIGHT) {
    if (vehicleLeftTimer == 0) vehicleLeftTimer = millis();
    if (millis() - vehicleLeftTimer >= VEHICLE_LEFT_DELAY) {
      clearIdentifiedVehicle();
      vehicleWasOnScale = false;
      vehicleLeftTimer = 0;
      stopRfidSearchSilently();
      Serial.println("RFID READY - WAITING FOR 0.500 kg");
    }
  } else if (currentWeight >= VEHICLE_LEFT_WEIGHT) {
    vehicleLeftTimer = 0;
  }
}

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, SCK);
  delay(1000);
  tareScale();

  loadVehicles();
  Serial.print("VEHICLES LOADED: "); Serial.println(vehicleCount);

  Serial2.begin(115200, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  stopRfidSearchSilently();
  Serial.println("RFID READY - WAITING FOR 0.500 kg");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/vehicles", HTTP_GET, handleVehiclesPage);
  server.on("/vehicles.json", HTTP_GET, handleVehiclesJson);
  server.on("/current-rfid", HTTP_GET, handleCurrentRfid);
  server.on("/vehicle/save", HTTP_POST, handleVehicleSave);
  server.on("/vehicle/delete", HTTP_POST, handleVehicleDelete);
  server.begin();

  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void loop() {
  handleRfidInput();
  handleRfidPolling();

  long raw = averageRead(20);
  currentWeight = (offset - raw) / calibrationFactor;

  if (fabs(currentWeight - lastWeight) < 0.005) {
    if (stableTimer == 0) stableTimer = millis();
    if (millis() - stableTimer > 2000) {
      stable = true;
      lastStableWeight = currentWeight;
    }
  } else {
    stable = false;
    stableTimer = 0;
  }

  updateVehicleCycle();
  lastWeight = currentWeight;
  server.handleClient();
}
