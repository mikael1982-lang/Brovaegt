#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "HX711.h"

#define DOUT 4
#define SCK 5

const int RFID_RX_PIN = 16;
const int RFID_TX_PIN = 17;
const byte SINGLE_POLL_CMD[7] = {0xAA,0x00,0x22,0x00,0x00,0x22,0xDD};
const byte STOP_MULTI_CMD[7] = {0xAA,0x00,0x28,0x00,0x00,0x28,0xDD};
const size_t RFID_MAX_FRAME_LENGTH = 64;
const unsigned long RFID_POLL_INTERVAL = 250;
const unsigned long RFID_AFTER_TAG_PAUSE = 450;
const unsigned long RFID_STABLE_FINISH_DELAY = 500;
const float RFID_TRIGGER_WEIGHT = 0.500;
const float VEHICLE_LEFT_WEIGHT = 0.050;
const unsigned long VEHICLE_LEFT_DELAY = 1500;
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
bool rfidSearching = false;
bool rfidStopSent = false;
unsigned long lastRfidPoll = 0;
unsigned long rfidPauseUntil = 0;
unsigned long rfidStableFinishTimer = 0;
bool vehicleWasOnScale = false;
unsigned long vehicleLeftTimer = 0;

const int MAX_SCANNED_TAGS = 6;
String scannedTags[MAX_SCANNED_TAGS];
int scannedTagCount = 0;

const int MAX_VEHICLES = 30;
struct VehicleEntry {
  String epc;
  String name;
  String owner;
  String role;      // "vehicle" or "attachment"
  float modelTare;
  float realTare;
  float maxGross;   // used from primary vehicle
};
VehicleEntry vehicles[MAX_VEHICLES];
int vehicleCount = 0;

String primaryName = "";
String primaryOwner = "";
String attachmentNames = "";
float combinedModelTare = 0.0;
float combinedRealTare = 0.0;
float primaryMaxGross = 0.0;
bool primaryKnown = false;
int unknownTagCount = 0;

long averageRead(int n){
  long long s = 0;
  for(int i=0;i<n;i++){
    while(!scale.is_ready()) delay(1);
    s += scale.read();
  }
  return s/n;
}

void tareScale(){ offset = averageRead(50); }

String cleanText(String value){
  value.trim();
  value.replace(";"," ");
  value.replace("\n"," ");
  value.replace("\r"," ");
  value.replace("\t"," ");
  return value;
}

String jsonEscape(const String& value){
  String out="";
  out.reserve(value.length()+8);
  for(size_t i=0;i<value.length();i++){
    char c=value[i];
    if(c=='\\' || c=='\"'){ out+='\\'; out+=c; }
    else if(c=='\n') out+="\\n";
    else if(c=='\r') out+="\\r";
    else out+=c;
  }
  return out;
}

void loadVehicles(){
  preferences.begin("vehicles",false);
  vehicleCount=preferences.getUInt("count",0);
  if(vehicleCount>MAX_VEHICLES) vehicleCount=MAX_VEHICLES;
  for(int i=0;i<vehicleCount;i++){
    String eKey="e"+String(i), nKey="n"+String(i), oKey="o"+String(i);
    String yKey="y"+String(i), tKey="t"+String(i), rKey="r"+String(i), gKey="g"+String(i);
    vehicles[i].epc=preferences.getString(eKey.c_str(),"");
    vehicles[i].name=preferences.getString(nKey.c_str(),"");
    vehicles[i].owner=preferences.getString(oKey.c_str(),"");
    vehicles[i].role=preferences.getString(yKey.c_str(),"vehicle");
    vehicles[i].modelTare=preferences.getFloat(tKey.c_str(),0.0);
    vehicles[i].realTare=preferences.getFloat(rKey.c_str(),0.0);
    vehicles[i].maxGross=preferences.getFloat(gKey.c_str(),0.0);
  }
}

void saveVehicles(){
  preferences.putUInt("count",vehicleCount);
  for(int i=0;i<MAX_VEHICLES;i++){
    String eKey="e"+String(i), nKey="n"+String(i), oKey="o"+String(i);
    String yKey="y"+String(i), tKey="t"+String(i), rKey="r"+String(i), gKey="g"+String(i);
    if(i<vehicleCount){
      preferences.putString(eKey.c_str(),vehicles[i].epc);
      preferences.putString(nKey.c_str(),vehicles[i].name);
      preferences.putString(oKey.c_str(),vehicles[i].owner);
      preferences.putString(yKey.c_str(),vehicles[i].role);
      preferences.putFloat(tKey.c_str(),vehicles[i].modelTare);
      preferences.putFloat(rKey.c_str(),vehicles[i].realTare);
      preferences.putFloat(gKey.c_str(),vehicles[i].maxGross);
    } else {
      preferences.remove(eKey.c_str()); preferences.remove(nKey.c_str()); preferences.remove(oKey.c_str());
      preferences.remove(yKey.c_str()); preferences.remove(tKey.c_str()); preferences.remove(rKey.c_str()); preferences.remove(gKey.c_str());
    }
  }
}

int findVehicleIndex(const String& epc){
  for(int i=0;i<vehicleCount;i++) if(vehicles[i].epc==epc) return i;
  return -1;
}

void clearScan(){
  scannedTagCount=0;
  for(int i=0;i<MAX_SCANNED_TAGS;i++) scannedTags[i]="";
  primaryName="";
  primaryOwner="";
  attachmentNames="";
  combinedModelTare=0.0;
  combinedRealTare=0.0;
  primaryMaxGross=0.0;
  primaryKnown=false;
  unknownTagCount=0;
}

bool tagAlreadyScanned(const String& epc){
  for(int i=0;i<scannedTagCount;i++) if(scannedTags[i]==epc) return true;
  return false;
}

void resolveCombination(){
  primaryName="";
  primaryOwner="";
  attachmentNames="";
  combinedModelTare=0.0;
  combinedRealTare=0.0;
  primaryMaxGross=0.0;
  primaryKnown=false;
  unknownTagCount=0;

  for(int i=0;i<scannedTagCount;i++){
    int idx=findVehicleIndex(scannedTags[i]);
    if(idx<0){ unknownTagCount++; continue; }
    combinedModelTare += vehicles[idx].modelTare;
    combinedRealTare += vehicles[idx].realTare;
    if(vehicles[idx].role=="vehicle"){
      if(!primaryKnown){
        primaryKnown=true;
        primaryName=vehicles[idx].name;
        primaryOwner=vehicles[idx].owner;
        primaryMaxGross=vehicles[idx].maxGross;
      }
    } else {
      if(attachmentNames.length()>0) attachmentNames += " + ";
      attachmentNames += vehicles[idx].name;
    }
  }
}

float calculatedRealLoad(){
  if(!primaryKnown) return 0.0;
  float modelLoad=currentWeight-combinedModelTare;
  if(modelLoad<0.0) modelLoad=0.0;
  return modelLoad*LOAD_SCALE_FACTOR;
}

float calculatedRealGross(){ return combinedRealTare + calculatedRealLoad(); }

String mainPage(){
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>
<meta name='apple-mobile-web-app-capable' content='yes'>
<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent'>
<meta name='apple-mobile-web-app-title' content='Brovægt'>
<title>Brovægt</title>
<style>
html,body{margin:0;padding:0;background:#151515;color:#fff;font-family:Arial,sans-serif;min-height:100%}.wrap{max-width:900px;margin:auto;padding:20px;text-align:center;box-sizing:border-box}h1{font-size:42px;margin:5px 0 12px;letter-spacing:2px}#measureStatus{font-size:26px;font-weight:bold;color:#ffd000;margin-bottom:20px}.vehicleCard{background:#242424;border:1px solid #555;border-radius:16px;padding:20px;margin-bottom:18px}#vehicle{font-size:38px;font-weight:bold}#owner,#attachment{font-size:21px;margin-top:8px;color:#ccc}.grid{display:table;width:100%;border-spacing:12px;table-layout:fixed}.cell{display:table-cell;background:#242424;border:1px solid #555;border-radius:16px;padding:18px 8px;vertical-align:middle}.label{font-size:18px;color:#bbb;margin-bottom:8px;text-transform:uppercase}.value{font-size:40px;font-weight:bold;white-space:nowrap}#result{font-size:42px;font-weight:bold;border-radius:16px;padding:18px;margin:12px 0 18px;background:#444}.ok{background:#245b2c!important}.warn{background:#9a6a00!important}.over{background:#9a2424!important}.unknown{background:#555!important}.small{font-size:20px;margin-top:6px}.adminLink{display:inline-block;margin-top:12px;padding:10px 18px;background:#333;color:#bbb;text-decoration:none;border-radius:8px;font-size:16px}@media(max-width:650px){.grid{display:block;border-spacing:0}.cell{display:block;margin-bottom:12px}.value{font-size:34px}#vehicle{font-size:32px}#result{font-size:34px}}
</style></head><body><div class='wrap'>
<h1>BROVÆGT</h1><div id='measureStatus'>VENTER</div>
<div class='vehicleCard'><div id='vehicle'>Intet køretøj</div><div id='owner'>Ejer: --</div><div id='attachment'></div></div>
<div class='grid'><div class='cell'><div class='label'>Egenvægt</div><div id='tare' class='value'>--</div></div><div class='cell'><div class='label'>Last</div><div id='load' class='value'>--</div></div><div class='cell'><div class='label'>Totalvægt</div><div id='gross' class='value'>--</div></div></div>
<div id='result' class='unknown'>VENTER PÅ KØRETØJ</div><div id='overBy' class='small'></div><a class='adminLink' href='/vehicles'>Register</a>
<script>
function byId(id){return document.getElementById(id)}
function formatKg(v){var n=Math.round(Number(v));var s=String(Math.abs(n));var out='';while(s.length>3){out='.'+s.substr(s.length-3)+out;s=s.substr(0,s.length-3)}out=s+out;if(n<0)out='-'+out;return out+' kg'}
function request(){var x=new XMLHttpRequest();x.open('GET','/status',true);x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){var p=x.responseText.split(';');var known=p[0]=='1';var onScale=p[1]=='1';var st=p[2]=='1';var unknown=Number(p[12]||0);byId('measureStatus').innerHTML=onScale?(st?'STABIL':'MÅLER'):'VENTER';byId('measureStatus').style.color=st?'#00ff00':'#ffd000';if(!onScale){byId('vehicle').innerHTML='Intet køretøj';byId('owner').innerHTML='Ejer: --';byId('attachment').innerHTML='';byId('tare').innerHTML='--';byId('load').innerHTML='--';byId('gross').innerHTML='--';byId('result').className='unknown';byId('result').innerHTML='VENTER PÅ KØRETØJ';byId('overBy').innerHTML='';return}if(!known){byId('vehicle').innerHTML='UKENDT KØRETØJ';byId('owner').innerHTML='Ejer: --';byId('attachment').innerHTML='';byId('tare').innerHTML='--';byId('load').innerHTML='--';byId('gross').innerHTML='--';byId('result').className='warn';byId('result').innerHTML='KØRETØJ IKKE REGISTRERET';byId('overBy').innerHTML=unknown>0?('Ukendte tags: '+unknown):'';return}byId('vehicle').innerHTML=p[3]||'Køretøj';byId('owner').innerHTML='Ejer: '+(p[4]||'--');byId('attachment').innerHTML=p[10]?('Tilkoblet: '+p[10]):'';byId('tare').innerHTML=formatKg(p[5]);byId('load').innerHTML=formatKg(p[6]);byId('gross').innerHTML=formatKg(p[7]);var max=Number(p[8]);var over=p[9]=='1';if(!st){byId('result').className='unknown';byId('result').innerHTML='MÅLER';byId('overBy').innerHTML=(Number(p[11])>1?('RFID tags fundet: '+p[11]):'')}else if(unknown>0){byId('result').className='warn';byId('result').innerHTML='UKENDT TILKOBLING';byId('overBy').innerHTML='Ukendte tags: '+unknown}else if(max<=0){byId('result').className='unknown';byId('result').innerHTML='INGEN VÆGTGRÆNSE';byId('overBy').innerHTML=''}else if(over){byId('result').className='over';byId('result').innerHTML='OVERLÆS';byId('overBy').innerHTML='Over med '+formatKg(Number(p[7])-max)+' · Tilladt '+formatKg(max)}else{byId('result').className='ok';byId('result').innerHTML='OK';byId('overBy').innerHTML='Tilladt totalvægt: '+formatKg(max)}}};x.send(null)}
setInterval(request,500);request();
</script></div></body></html>)rawliteral";
}

String vehiclesPage(){
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><meta name='apple-mobile-web-app-capable' content='yes'><title>Register</title>
<style>
body{background:#202020;color:white;font-family:Arial;margin:0;padding:18px}.wrap{max-width:760px;margin:auto}h1{text-align:center}.back{display:inline-block;padding:10px 14px;background:#444;color:white;text-decoration:none;border-radius:8px;margin-bottom:12px}.box{padding:18px;border:1px solid #555;border-radius:14px;background:#292929;margin-bottom:18px}label{display:block;margin-top:12px;font-size:17px}input,select{width:100%;box-sizing:border-box;padding:12px;margin-top:5px;border-radius:8px;border:1px solid #777;font-size:18px}input[readonly]{background:#1d1d1d;color:#ddd}.weightRow{display:table;width:100%;table-layout:fixed}.weightField,.weightButton{display:table-cell;vertical-align:bottom}.weightField{padding-right:8px}.weightButton{width:220px}.weightButton button{width:100%}button{padding:12px 16px;margin-top:12px;border:0;border-radius:8px;font-size:17px;font-weight:bold}.primary{background:#45a049;color:white;width:100%}.secondary{background:#555;color:white}.capture{background:#306a9c;color:white}.danger{background:#a43b3b;color:white}.vehicleRow{border-top:1px solid #555;padding:15px 0}.vehicleName{font-size:21px;font-weight:bold}.vehicleMeta{font-size:15px;color:#ccc;margin-top:5px}.vehicleEpc{font-family:monospace;word-break:break-all;font-size:12px;color:#aaa;margin-top:5px}#message{text-align:center;min-height:22px;margin-top:10px;color:#ffd000}.hint{font-size:14px;color:#aaa;margin-top:6px}.tagBtn{display:block;width:100%;text-align:left;font-family:monospace}@media(max-width:600px){.weightRow,.weightField,.weightButton{display:block;width:100%}.weightField{padding-right:0}}
</style></head><body><div class='wrap'>
<a class='back' href='/'>← Brovægt</a><h1>Køretøjsregister</h1>
<div class='box'><h2>Tags fundet på broen</h2><div id='currentTags'>Ingen tags endnu.</div></div>
<div class='box'>
<label>RFID / EPC</label><input id='regEpc' type='text' placeholder='Vælg et fundet tag eller indsæt EPC'>
<label>Type</label><select id='regRole'><option value='vehicle'>Hovedkøretøj / traktor</option><option value='attachment'>Trailer / vogn / container</option></select>
<label>Navn</label><input id='regName' type='text' placeholder='F.eks. Kroghejs eller Tiptrailer 2'>
<label>Ejer</label><input id='regOwner' type='text' placeholder='F.eks. Mikael'>
<div class='weightRow'><div class='weightField'><label>Model egenvægt (kg)</label><input id='regModelTare' type='text' readonly placeholder='Vent på stabil vægt'></div><div class='weightButton'><button id='useWeight' class='capture' type='button'>Brug aktuel vægt</button></div></div>
<div class='hint'>Ved registrering af en enkelt enhed skal enheden stå tom på broen. For vogne/containere kan vi senere lave differencemåling mod et kendt hovedkøretøj.</div>
<label>Virkelig egenvægt (kg)</label><input id='regRealTare' type='text' inputmode='numeric' placeholder='F.eks. 15500'>
<label>Maks. tilladt totalvægt (kg)</label><input id='regMaxGross' type='text' inputmode='numeric' placeholder='Bruges på hovedkøretøj; 0 for tilkoblet enhed'>
<button id='saveVehicle' class='primary' type='button'>Gem</button><div id='message'></div>
</div>
<div class='box'><h2>Registrerede enheder</h2><div id='vehicleList'></div></div>
<script>
function byId(id){return document.getElementById(id)}
function trimText(s){return String(s).replace(/^\s+|\s+$/g,'')}
function setMessage(msg){byId('message').innerHTML=msg}
function enc(v){return encodeURIComponent(v).replace(/%20/g,'+')}
function ajax(method,url,body,ok,fail){var x=new XMLHttpRequest();x.open(method,url,true);if(method=='POST')x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');x.onreadystatechange=function(){if(x.readyState==4){if(x.status>=200&&x.status<300){if(ok)ok(x.responseText)}else{if(fail)fail(x.responseText||('Fejl '+x.status))}}};x.send(body||null)}
function refreshTags(){ajax('GET','/current-tags',null,function(txt){var list;try{list=JSON.parse(txt)}catch(e){return}var box=byId('currentTags');box.innerHTML='';if(!list.length){box.innerHTML='Ingen tags endnu.';return}for(var i=0;i<list.length;i++){var b=document.createElement('button');b.className='secondary tagBtn';b.type='button';b.appendChild(document.createTextNode(list[i]));b.onclick=(function(epc){return function(){byId('regEpc').value=epc;setMessage('RFID valgt.')};})(list[i]);box.appendChild(b)}})}
function captureWeight(){setMessage('Kontrollerer vægten...');ajax('GET','/current-weight',null,function(t){var p=t.split(';');if(p[0]!='1'){setMessage('Vent på STABIL vægt.');return}var w=Number(p[1]);if(isNaN(w)||w<0.500){setMessage('Der skal stå mindst 0,500 kg på broen.');return}byId('regModelTare').value=w.toFixed(3);setMessage('Model egenvægt målt: '+w.toFixed(3)+' kg')},function(e){setMessage(e)})}
function editVehicle(v){byId('regEpc').value=v.epc;byId('regRole').value=v.role||'vehicle';byId('regName').value=v.name;byId('regOwner').value=v.owner;byId('regModelTare').value=Number(v.modelTare).toFixed(3);byId('regRealTare').value=Math.round(Number(v.realTare));byId('regMaxGross').value=Math.round(Number(v.maxGross));setMessage('Enhed klar til redigering.');window.scrollTo(0,0)}
function deleteVehicle(epc,name){if(!confirm('Slet '+name+'?'))return;ajax('POST','/vehicle/delete','epc='+enc(epc),function(){setMessage('Enhed slettet.');refreshVehicles()},function(e){setMessage(e)})}
function refreshVehicles(){ajax('GET','/vehicles.json',null,function(txt){var list;try{list=JSON.parse(txt)}catch(e){setMessage('Kunne ikke læse registeret.');return}var box=byId('vehicleList');box.innerHTML='';if(!list.length){box.innerHTML='Ingen enheder gemt endnu.';return}for(var i=0;i<list.length;i++){(function(v){var row=document.createElement('div');row.className='vehicleRow';var n=document.createElement('div');n.className='vehicleName';n.appendChild(document.createTextNode(v.name+' · '+((v.role||'vehicle')=='vehicle'?'Hovedkøretøj':'Tilkoblet enhed')));row.appendChild(n);var m=document.createElement('div');m.className='vehicleMeta';m.appendChild(document.createTextNode('Ejer: '+(v.owner||'--')+' · Model egenvægt: '+Number(v.modelTare).toFixed(3)+' kg · Virkelig egenvægt: '+Math.round(Number(v.realTare))+' kg · Maks: '+Math.round(Number(v.maxGross))+' kg'));row.appendChild(m);var e=document.createElement('div');e.className='vehicleEpc';e.appendChild(document.createTextNode(v.epc));row.appendChild(e);var eb=document.createElement('button');eb.className='secondary';eb.type='button';eb.innerHTML='Redigér';eb.onclick=function(){editVehicle(v)};row.appendChild(eb);var db=document.createElement('button');db.className='danger';db.type='button';db.innerHTML='Slet';db.onclick=function(){deleteVehicle(v.epc,v.name)};row.appendChild(db);box.appendChild(row)})(list[i])}})}
function num(id){return trimText(byId(id).value).replace(',','.')}
function saveVehicle(){var epc=trimText(byId('regEpc').value).toUpperCase();var role=byId('regRole').value;var name=trimText(byId('regName').value);var owner=trimText(byId('regOwner').value);var mt=num('regModelTare');var rt=num('regRealTare');var mg=num('regMaxGross');if(!epc||!name||!owner||mt===''||rt===''||mg===''){setMessage('Udfyld alle felter og mål model-egenvægten.');return}if(isNaN(Number(mt))||isNaN(Number(rt))||isNaN(Number(mg))||Number(mt)<0||Number(rt)<0||Number(mg)<0){setMessage('Vægte skal være gyldige positive tal.');return}var body='epc='+enc(epc)+'&role='+enc(role)+'&name='+enc(name)+'&owner='+enc(owner)+'&modelTare='+enc(mt)+'&realTare='+enc(rt)+'&maxGross='+enc(mg);setMessage('Gemmer...');ajax('POST','/vehicle/save',body,function(){setMessage('Enhed gemt.');byId('regEpc').value='';byId('regName').value='';byId('regOwner').value='';byId('regModelTare').value='';byId('regRealTare').value='';byId('regMaxGross').value='';refreshVehicles()},function(e){setMessage(e)})}
byId('useWeight').onclick=captureWeight;byId('saveVehicle').onclick=saveVehicle;setInterval(refreshTags,700);refreshTags();refreshVehicles();
</script></div></body></html>)rawliteral";
}

void handleRoot(){ server.send(200,"text/html",mainPage()); }
void handleVehiclesPage(){ server.send(200,"text/html",vehiclesPage()); }

void handleStatus(){
  bool onScale=currentWeight>=RFID_TRIGGER_WEIGHT || vehicleWasOnScale;
  resolveCombination();
  float load=calculatedRealLoad();
  float gross=calculatedRealGross();
  bool overload=primaryKnown && stable && primaryMaxGross>0.0 && gross>primaryMaxGross;
  server.send(200,"text/plain",
    String(primaryKnown?1:0)+";"+String(onScale?1:0)+";"+String(stable?1:0)+";"+
    (primaryKnown?primaryName:"")+";"+(primaryKnown?primaryOwner:"")+";"+
    String(primaryKnown?combinedRealTare:0.0,0)+";"+String(primaryKnown?load:0.0,0)+";"+
    String(primaryKnown?gross:0.0,0)+";"+String(primaryKnown?primaryMaxGross:0.0,0)+";"+String(overload?1:0)+";"+
    attachmentNames+";"+String(scannedTagCount)+";"+String(unknownTagCount));
}

void handleCurrentTags(){
  String json="[";
  for(int i=0;i<scannedTagCount;i++){
    if(i>0) json+=",";
    json+="\""+jsonEscape(scannedTags[i])+"\"";
  }
  json+="]";
  server.send(200,"application/json",json);
}

void handleCurrentWeight(){ server.send(200,"text/plain",String(stable?1:0)+";"+String(currentWeight,3)); }

void handleVehiclesJson(){
  String json="[";
  for(int i=0;i<vehicleCount;i++){
    if(i>0) json+=",";
    json+="{\"epc\":\""+jsonEscape(vehicles[i].epc)+"\",\"name\":\""+jsonEscape(vehicles[i].name)+"\",\"owner\":\""+jsonEscape(vehicles[i].owner)+"\",\"role\":\""+jsonEscape(vehicles[i].role)+"\",\"modelTare\":"+String(vehicles[i].modelTare,3)+",\"realTare\":"+String(vehicles[i].realTare,0)+",\"maxGross\":"+String(vehicles[i].maxGross,0)+"}";
  }
  json+="]";
  server.send(200,"application/json",json);
}

void handleVehicleSave(){
  if(!server.hasArg("epc")||!server.hasArg("role")||!server.hasArg("name")||!server.hasArg("owner")||!server.hasArg("modelTare")||!server.hasArg("realTare")||!server.hasArg("maxGross")){
    server.send(400,"text/plain","Mangler et eller flere felter."); return;
  }
  String epc=cleanText(server.arg("epc")); epc.toUpperCase();
  String role=cleanText(server.arg("role")); if(role!="attachment") role="vehicle";
  String name=cleanText(server.arg("name"));
  String owner=cleanText(server.arg("owner"));
  String modelText=cleanText(server.arg("modelTare")); modelText.replace(",",".");
  String realText=cleanText(server.arg("realTare")); realText.replace(",",".");
  String maxText=cleanText(server.arg("maxGross")); maxText.replace(",",".");
  if(epc.length()==0||name.length()==0||owner.length()==0||modelText.length()==0||realText.length()==0||maxText.length()==0){ server.send(400,"text/plain","Udfyld alle felter."); return; }
  float modelTare=modelText.toFloat(), realTare=realText.toFloat(), maxGross=maxText.toFloat();
  if(modelTare<0.0||realTare<0.0||maxGross<0.0){ server.send(400,"text/plain","Vægte må ikke være negative."); return; }
  int index=findVehicleIndex(epc);
  if(index<0){ if(vehicleCount>=MAX_VEHICLES){ server.send(400,"text/plain","Registeret er fuldt."); return; } index=vehicleCount++; }
  vehicles[index].epc=epc; vehicles[index].role=role; vehicles[index].name=name; vehicles[index].owner=owner;
  vehicles[index].modelTare=modelTare; vehicles[index].realTare=realTare; vehicles[index].maxGross=maxGross;
  saveVehicles(); resolveCombination();
  Serial.print("UNIT SAVED: "); Serial.print(epc); Serial.print(" / "); Serial.print(role); Serial.print(" / "); Serial.println(name);
  server.send(200,"text/plain","OK");
}

void handleVehicleDelete(){
  if(!server.hasArg("epc")){ server.send(400,"text/plain","Mangler RFID."); return; }
  String epc=cleanText(server.arg("epc")); epc.toUpperCase();
  int index=findVehicleIndex(epc);
  if(index<0){ server.send(404,"text/plain","Enheden findes ikke."); return; }
  for(int i=index;i<vehicleCount-1;i++) vehicles[i]=vehicles[i+1];
  vehicleCount--; saveVehicles(); resolveCombination();
  Serial.print("UNIT DELETED: "); Serial.println(epc);
  server.send(200,"text/plain","OK");
}

void resetRfidFrame(){ rfidFrameLength=0; }

bool rfidFrameChecksumValid(){
  byte checksum=0;
  for(size_t i=1;i<rfidFrameLength-2;i++) checksum+=rfidFrame[i];
  return checksum==rfidFrame[rfidFrameLength-2];
}

void startRfidSearch(){
  clearScan();
  rfidSearching=true;
  rfidStopSent=false;
  lastRfidPoll=0;
  rfidPauseUntil=0;
  rfidStableFinishTimer=0;
  resetRfidFrame();
  Serial.println("RFID MULTI-TAG SEARCH STARTED - WEIGHT >= 0.500 kg");
}

void stopRfidSearch(bool announce){
  rfidSearching=false;
  lastRfidPoll=0;
  rfidPauseUntil=0;
  rfidStableFinishTimer=0;
  resetRfidFrame();
  if(!rfidStopSent){
    Serial2.write(STOP_MULTI_CMD,sizeof(STOP_MULTI_CMD));
    rfidStopSent=true;
    if(announce){ Serial.print("RFID SEARCH FINISHED - "); Serial.print(scannedTagCount); Serial.println(" TAG(S)"); }
  }
}

void handleRfidPolling(){
  if(!rfidSearching) return;
  unsigned long now=millis();
  if(now<rfidPauseUntil) return;
  if(lastRfidPoll==0 || now-lastRfidPoll>=RFID_POLL_INTERVAL){
    Serial2.write(SINGLE_POLL_CMD,sizeof(SINGLE_POLL_CMD));
    lastRfidPoll=now;
  }
}

void processRfidTagFrame(){
  int epcBytes=(((rfidFrame[6]<<8)|rfidFrame[7])>>11&0x1F)*2;
  if(epcBytes<=0 || 8+epcBytes>(int)rfidFrameLength-4) return;
  String epc="";
  for(int i=0;i<epcBytes;i++){ if(rfidFrame[8+i]<0x10) epc+="0"; epc+=String(rfidFrame[8+i],HEX); }
  epc.toUpperCase();
  if(!tagAlreadyScanned(epc) && scannedTagCount<MAX_SCANNED_TAGS){
    scannedTags[scannedTagCount++]=epc;
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] RFID TAG "); Serial.print(scannedTagCount); Serial.print(": "); Serial.println(epc);
    resolveCombination();
  }
  rfidPauseUntil=millis()+RFID_AFTER_TAG_PAUSE;
}

void handleRfidInput(){
  while(Serial2.available()>0){
    byte rfidByte=Serial2.read();
    if(rfidFrameLength==0){ if(rfidByte==0xAA) rfidFrame[rfidFrameLength++]=rfidByte; continue; }
    if(rfidByte==0xAA){ rfidFrame[0]=rfidByte; rfidFrameLength=1; continue; }
    if(rfidFrameLength>=RFID_MAX_FRAME_LENGTH){ resetRfidFrame(); continue; }
    rfidFrame[rfidFrameLength++]=rfidByte;
    if(rfidFrameLength>=5){
      size_t expectedLength=((size_t)rfidFrame[3]<<8 | rfidFrame[4])+7;
      if(expectedLength>RFID_MAX_FRAME_LENGTH) resetRfidFrame();
      else if(rfidFrameLength==expectedLength){
        if(rfidFrame[rfidFrameLength-1]==0xDD && rfidFrame[1]==0x02 && rfidFrame[2]==0x22 && rfidFrameChecksumValid()) processRfidTagFrame();
        resetRfidFrame();
      } else if(rfidFrameLength>expectedLength) resetRfidFrame();
    }
  }
}

void updateVehicleCycle(){
  if(!vehicleWasOnScale && currentWeight>=RFID_TRIGGER_WEIGHT){
    vehicleWasOnScale=true;
    vehicleLeftTimer=0;
    startRfidSearch();
  }

  if(rfidSearching && stable && scannedTagCount>0){
    if(rfidStableFinishTimer==0) rfidStableFinishTimer=millis();
    if(millis()-rfidStableFinishTimer>=RFID_STABLE_FINISH_DELAY) stopRfidSearch(true);
  } else if(!stable){
    rfidStableFinishTimer=0;
  }

  if(vehicleWasOnScale && currentWeight<VEHICLE_LEFT_WEIGHT){
    if(vehicleLeftTimer==0) vehicleLeftTimer=millis();
    if(millis()-vehicleLeftTimer>=VEHICLE_LEFT_DELAY){
      if(rfidSearching) stopRfidSearch(false);
      clearScan();
      vehicleWasOnScale=false;
      vehicleLeftTimer=0;
      rfidStopSent=false;
      Serial.println("RFID READY - WAITING FOR 0.500 kg");
    }
  } else if(currentWeight>=VEHICLE_LEFT_WEIGHT){
    vehicleLeftTimer=0;
  }
}

void setup(){
  Serial.begin(115200);
  scale.begin(DOUT,SCK);
  delay(1000);
  tareScale();
  loadVehicles();
  Serial.print("UNITS LOADED: "); Serial.println(vehicleCount);
  Serial2.begin(115200,SERIAL_8N1,RFID_RX_PIN,RFID_TX_PIN);
  clearScan();
  Serial.println("RFID READY - WAITING FOR 0.500 kg");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);
  server.on("/",HTTP_GET,handleRoot);
  server.on("/status",HTTP_GET,handleStatus);
  server.on("/vehicles",HTTP_GET,handleVehiclesPage);
  server.on("/vehicles.json",HTTP_GET,handleVehiclesJson);
  server.on("/current-tags",HTTP_GET,handleCurrentTags);
  server.on("/current-weight",HTTP_GET,handleCurrentWeight);
  server.on("/vehicle/save",HTTP_POST,handleVehicleSave);
  server.on("/vehicle/delete",HTTP_POST,handleVehicleDelete);
  server.begin();
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void loop(){
  handleRfidInput();
  handleRfidPolling();
  long raw=averageRead(20);
  currentWeight=(offset-raw)/calibrationFactor;
  if(fabs(currentWeight-lastWeight)<0.005){
    if(stableTimer==0) stableTimer=millis();
    if(millis()-stableTimer>2000){ stable=true; lastStableWeight=currentWeight; }
  } else {
    stable=false;
    stableTimer=0;
  }
  updateVehicleCycle();
  lastWeight=currentWeight;
  server.handleClient();
}
