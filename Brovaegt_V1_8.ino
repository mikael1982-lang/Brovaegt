// Brovægt V1.8 TEST — based on working V1.7.6. RFID trigger unchanged.
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
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
const unsigned long RFID_SCAN_DURATION = 10000;
const float RFID_TRIGGER_WEIGHT = 0.500;
const float VEHICLE_LEFT_WEIGHT = 0.200;
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
bool rfidManualSearch = false;
bool rfidFinished = false;
unsigned long rfidFrameStarted = 0;
bool rfidStopSent = false;
unsigned long lastRfidPoll = 0;
unsigned long rfidPauseUntil = 0;
unsigned long rfidSearchStarted = 0;
bool vehicleWasOnScale = false;
unsigned long vehicleLeftTimer = 0;

const int MAX_SCANNED_TAGS = 6;
String scannedTags[MAX_SCANNED_TAGS];
int scannedTagCount = 0;
// Latest RFID result is independent of the current weighing combination.
String resultTags[MAX_SCANNED_TAGS];
int resultTagCount = 0;

// Pladser og dagslog gemmes lokalt. Dato/klokkeslæt synkroniseres fra iPad.
const int MAX_PLACES=24;
String places[MAX_PLACES];
int placeCount=0;
int pickup=-1, dropoff=-1;
bool journeySaved=false;
bool clockReady=false;
uint32_t clockEpoch=0, clockAt=0;
String dateNow(){
 if(!clockReady)return "";
 time_t t=(time_t)(clockEpoch+(millis()-clockAt)/1000);
 struct tm *p=gmtime(&t);
 if(!p)return "";
 char b[12];strftime(b,sizeof(b),"%Y-%m-%d",p);return String(b);
}
String timeNow(){
 if(!clockReady)return "";
 time_t t=(time_t)(clockEpoch+(millis()-clockAt)/1000);
 struct tm *p=gmtime(&t);
 if(!p)return "";
 char b[12];strftime(b,sizeof(b),"%H:%M:%S",p);return String(b);
}
void loadPlaces(){
 Preferences p;p.begin("places18",true);
 placeCount=min((int)p.getUInt("count",0),MAX_PLACES);
 for(int i=0;i<placeCount;i++)places[i]=p.getString(("p"+String(i)).c_str(),"");
 p.end();
}
void savePlaces(){
 Preferences p;p.begin("places18",false);
 p.putUInt("count",placeCount);
 for(int i=0;i<placeCount;i++)p.putString(("p"+String(i)).c_str(),places[i]);
 p.end();
}
bool logStorageReady=false;
String safeCsv(String s){s.replace(";"," ");s.replace("|"," ");s.replace("\r"," ");s.replace("\n"," ");return s;}
bool appendLog(String line){if(!logStorageReady)return false;File f=LittleFS.open("/weighings.csv","a");if(!f)return false;size_t n=f.print(line);f.close();return n==line.length();}
const int MAX_VEHICLES = 30;
struct VehicleEntry {
  String epc;
  String name;
  String owner;
  String role;
  float modelTare;
  float realTare;
  float maxGross;
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
return R"V18HTML(<!doctype html><html lang="da"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1"><title>Brovægt</title><style>
body{background:#171717;color:white;font-family:Arial;margin:0}main{max-width:920px;margin:auto;padding:14px;text-align:center}h1{margin:10px}#state{border-radius:15px;padding:18px;font-size:34px;font-weight:bold;background:#167b43}.yellow{background:#d9a321!important;color:#201900}.green{background:#167b43!important;color:white}.red{background:#9c2929!important}.card{background:#292929;border:1px solid #555;border-radius:14px;padding:14px;margin:12px 0}#big{font-size:clamp(32px,7vw,62px);font-weight:bold}#small{font-size:17px;color:#ddd;margin-top:6px}.muted{color:#bbb}h2{font-size:21px}.buttons{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}button{min-height:53px;border:1px solid #777;border-radius:9px;background:#484848;color:white;font-size:16px;font-weight:bold;overflow-wrap:anywhere}button.selected{background:#167b43;border-color:#54cc82}button:disabled{opacity:.25}a{color:white;display:inline-block;padding:12px;background:#444;border-radius:8px;text-decoration:none;margin:4px}@media(max-width:500px){.buttons{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body><main><h1>BROVÆGT</h1><div id="state" class="green">KLAR</div>
<div class="card"><h2 id="vehicle">Intet køretøj</h2><div id="owner" class="muted"></div><div id="attachment" class="muted"></div><div id="big">0 kg</div><div id="small">Faktisk last: --</div><div id="details" class="muted"></div></div>
<div class="card"><h2>Afhentningssted</h2><div id="from" class="buttons"></div></div>
<div class="card"><h2>Aflæsningssted</h2><div id="to" class="buttons"></div></div>
<div id="note" class="muted"></div><nav><a href="/vehicles">Køretøjsregister</a><a href="/places">Pladsregister</a><a href="/report">Dagsrapport</a></nav>
</main><script>
var active=false,scan=false,stable=false,known=false,from=-1,to=-1,places=[],busy=false;
function el(id){return document.getElementById(id)}
function fmt(n){return Math.round(Number(n)||0).toLocaleString('da-DK')+' kg'}
function req(method,url,body,cb){var x=new XMLHttpRequest();x.open(method,url,true);x.timeout=3000;if(method=='POST')x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');x.onload=function(){if(cb)cb(x.status,x.responseText)};x.send(body||null)}
function draw(){['from','to'].forEach(function(side){var box=el(side);box.innerHTML='';places.forEach(function(name,i){var b=document.createElement('button');b.textContent=name;b.className=(side=='from'?from:to)==i?'selected':'';b.disabled=!active||(side=='from'?to:from)==i;b.onclick=function(){req('POST','/select','side='+side+'&index='+i,function(code){if(code!=200)el('note').textContent='Valget kunne ikke gemmes';else refresh()})};box.appendChild(b)})})}
function refresh(){if(busy)return;busy=true;req('GET','/state.json',null,function(code,body){busy=false;if(code!=200)return;var p;try{p=JSON.parse(body)}catch(e){return}active=p.active;scan=p.scanning;stable=p.stable;known=p.known;from=p.from;to=p.to;places=p.places;el('vehicle').textContent=!active?'Intet køretøj':known?p.vehicle:'UMÆRKET / UKENDT KØRETØJ';el('owner').textContent=known?'Ejer: '+p.owner:'';el('attachment').textContent=p.attachment?'Tilkoblet: '+p.attachment:'';el('big').textContent=fmt(p.display);el('small').textContent=known?'Faktisk last: '+Number(p.modelLoad).toFixed(3).replace('.',',')+' kg':'Omregnet bruttovægt (ingen fratrukket egenvægt)';el('details').textContent=known?'Egenvægt: '+fmt(p.tare)+' · Last: '+fmt(p.load)+' · Total: '+fmt(p.gross):'';el('note').textContent=p.saved?'Transport registreret i dagsrapporten':active&&!known?'Umærkede køretøjer gemmes ikke i dagsrapporten':'';draw()})}
function paint(){var node=el('state');if(!active){node.textContent='KLAR';node.className='green'}else if(!stable||scan){node.textContent=(Math.floor(Date.now()/500)%2==0)?'VEJER':'SCANNER';node.className='yellow'}else{node.textContent='STABIL';node.className='green'}}
req('POST','/clock','epoch='+Math.floor(Date.now()/1000),function(){});
setInterval(function(){refresh()},600);setInterval(paint,100);refresh();
</script></body></html>)V18HTML";
}

String vehiclesPage(){
return R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><meta name='apple-mobile-web-app-capable' content='yes'><title>Register</title>
<style>
body{background:#202020;color:white;font-family:Arial;margin:0;padding:18px}.wrap{max-width:760px;margin:auto}h1{text-align:center}.back{display:inline-block;padding:10px 14px;background:#444;color:white;text-decoration:none;border-radius:8px;margin-bottom:12px}.box{padding:18px;border:1px solid #555;border-radius:14px;background:#292929;margin-bottom:18px}label{display:block;margin-top:12px;font-size:17px}input,select{width:100%;box-sizing:border-box;padding:12px;margin-top:5px;border-radius:8px;border:1px solid #777;font-size:18px}input[readonly]{background:#1d1d1d;color:#ddd}.weightRow{display:table;width:100%;table-layout:fixed}.weightField,.weightButton{display:table-cell;vertical-align:bottom}.weightField{padding-right:8px}.weightButton{width:220px}.weightButton button{width:100%}button{padding:12px 16px;margin-top:12px;border:0;border-radius:8px;font-size:17px;font-weight:bold}.primary{background:#45a049;color:white;width:100%}.secondary{background:#555;color:white}.capture{background:#306a9c;color:white}.danger{background:#a43b3b;color:white}.vehicleRow{border-top:1px solid #555;padding:15px 0}.vehicleName{font-size:21px;font-weight:bold}.vehicleMeta{font-size:15px;color:#ccc;margin-top:5px}.vehicleEpc{font-family:monospace;word-break:break-all;font-size:12px;color:#aaa;margin-top:5px}#message{text-align:center;min-height:22px;margin-top:10px;color:#ffd000}.hint{font-size:14px;color:#aaa;margin-top:6px}.tagBtn{display:block;width:100%;text-align:left;font-family:monospace}@media(max-width:600px){.weightRow,.weightField,.weightButton{display:block;width:100%}.weightField{padding-right:0}}
</style></head><body><div class='wrap'>
<a class='back' href='/'>← Brovægt</a><h1>Køretøjsregister</h1>
<div class='box'><h2>Tags fundet på broen</h2><div id='rfidStatus'>RFID: KLAR</div><button id='testRfid' class='secondary' type='button'>TEST RFID</button><div id='currentTags'>Ingen tags endnu.</div></div>
<div class='box'>
<label>RFID / EPC</label><input id='regEpc' type='text' placeholder='Vælg et fundet tag eller indsæt EPC'>
<label>Type</label><select id='regRole'><option value='vehicle'>Hovedkøretøj / traktor</option><option value='attachment'>Trailer / vogn / container</option></select>
<label>Navn</label><input id='regName' type='text' placeholder='F.eks. Kroghejs eller Tiptrailer 2'>
<label>Ejer</label><input id='regOwner' type='text' placeholder='F.eks. Mikael'>
<div class='weightRow'><div class='weightField'><label>Model egenvægt (kg)</label><input id='regModelTare' type='text' readonly placeholder='Vent på stabil vægt'></div><div class='weightButton'><button id='useWeight' class='capture' type='button'>Brug aktuel vægt</button></div></div>
<div class='hint'>Ved registrering af en enkelt enhed skal enheden stå tom på broen.</div>
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
function captureWeight(){setMessage('Kontrollerer vægten...');ajax('GET','/current-weight',null,function(t){var p=t.split(';');if(p[0]!='1'){setMessage('Vent på STABIL vægt.');return}var w=Number(p[1]);if(isNaN(w)||w<0.500){setMessage('Der skal stå mindst 0,500 kg på broen.');return}byId('regModelTare').value=w.toFixed(3);setMessage('Model egenvægt målt: '+w.toFixed(3)+' kg')},function(e){setMessage(e)})}
function editVehicle(v){byId('regEpc').value=v.epc;byId('regRole').value=v.role||'vehicle';byId('regName').value=v.name;byId('regOwner').value=v.owner;byId('regModelTare').value=Number(v.modelTare).toFixed(3);byId('regRealTare').value=Math.round(Number(v.realTare));byId('regMaxGross').value=Math.round(Number(v.maxGross));setMessage('Enhed klar til redigering.');window.scrollTo(0,0)}
function deleteVehicle(epc,name){if(!confirm('Slet '+name+'?'))return;ajax('POST','/vehicle/delete','epc='+enc(epc),function(){setMessage('Enhed slettet.');refreshVehicles()},function(e){setMessage(e)})}
function refreshVehicles(){ajax('GET','/vehicles.json',null,function(txt){var list;try{list=JSON.parse(txt)}catch(e){setMessage('Kunne ikke læse registeret.');return}var box=byId('vehicleList');box.innerHTML='';if(!list.length){box.innerHTML='Ingen enheder gemt endnu.';return}for(var i=0;i<list.length;i++){(function(v){var row=document.createElement('div');row.className='vehicleRow';var n=document.createElement('div');n.className='vehicleName';n.appendChild(document.createTextNode(v.name+' · '+((v.role||'vehicle')=='vehicle'?'Hovedkøretøj':'Tilkoblet enhed')));row.appendChild(n);var m=document.createElement('div');m.className='vehicleMeta';m.appendChild(document.createTextNode('Ejer: '+(v.owner||'--')+' · Model egenvægt: '+Number(v.modelTare).toFixed(3)+' kg · Virkelig egenvægt: '+Math.round(Number(v.realTare))+' kg · Maks: '+Math.round(Number(v.maxGross))+' kg'));row.appendChild(m);var e=document.createElement('div');e.className='vehicleEpc';e.appendChild(document.createTextNode(v.epc));row.appendChild(e);var eb=document.createElement('button');eb.className='secondary';eb.type='button';eb.innerHTML='Redigér';eb.onclick=function(){editVehicle(v)};row.appendChild(eb);var db=document.createElement('button');db.className='danger';db.type='button';db.innerHTML='Slet';db.onclick=function(){deleteVehicle(v.epc,v.name)};row.appendChild(db);box.appendChild(row)})(list[i])}})}
function num(id){return trimText(byId(id).value).replace(',','.')}
function saveVehicle(){var epc=trimText(byId('regEpc').value).toUpperCase();var role=byId('regRole').value;var name=trimText(byId('regName').value);var owner=trimText(byId('regOwner').value);var mt=num('regModelTare');var rt=num('regRealTare');var mg=num('regMaxGross');if(!epc||!name||!owner||mt===''||rt===''||mg===''){setMessage('Udfyld alle felter og mål model-egenvægten.');return}if(isNaN(Number(mt))||isNaN(Number(rt))||isNaN(Number(mg))||Number(mt)<0||Number(rt)<0||Number(mg)<0){setMessage('Vægte skal være gyldige positive tal.');return}var body='epc='+enc(epc)+'&role='+enc(role)+'&name='+enc(name)+'&owner='+enc(owner)+'&modelTare='+enc(mt)+'&realTare='+enc(rt)+'&maxGross='+enc(mg);setMessage('Gemmer...');ajax('POST','/vehicle/save',body,function(){setMessage('Enhed gemt.');byId('regEpc').value='';byId('regName').value='';byId('regOwner').value='';byId('regModelTare').value='';byId('regRealTare').value='';byId('regMaxGross').value='';refreshVehicles()},function(e){setMessage(e)})}
byId('useWeight').onclick=captureWeight;byId('saveVehicle').onclick=saveVehicle;byId('testRfid').onclick=function(){byId('testRfid').disabled=true;ajax('POST','/rfid-test',null,function(txt){renderRfid(JSON.parse(txt))},function(e){setMessage(e);byId('testRfid').disabled=false})};refreshVehicles();

var rfidBusy=false;
function renderRfid(data){
 byId('rfidStatus').textContent='RFID: '+data.state+(data.state=='SCANNER'?' · '+data.remaining+' sek.':data.state=='FÆRDIG'?' · '+data.tags.length+' unikke tags':'');
 var test=byId('testRfid');if(test)test.disabled=data.state=='SCANNER';
 var box=byId('currentTags');box.innerHTML='';
 if(!data.tags.length){box.textContent='Ingen tags endnu.';return}
 for(var i=0;i<data.tags.length;i++){(function(tag){
  var row=document.createElement(test?'button':'div');row.className='secondary tagBtn';
  if(test){row.type='button';row.onclick=function(){byId('regEpc').value=tag.epc;setMessage('RFID valgt.')}}
  row.style.wordBreak='break-all';row.textContent=tag.epc+' · '+(tag.known?tag.name:'Ikke registreret')+' · '+(tag.known?'KENDT':'UKENDT TAG');box.appendChild(row);
 })(data.tags[i])}
}
function refreshRfid(){
 if(rfidBusy)return;rfidBusy=true;
 var x=new XMLHttpRequest();x.open('GET','/rfid-status?t='+new Date().getTime(),true);x.timeout=3000;
 var done=false;function finish(){if(done)return;done=true;rfidBusy=false;setTimeout(refreshRfid,500)}
 x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){try{renderRfid(JSON.parse(x.responseText))}catch(e){byId('rfidStatus').textContent='RFID: svar kunne ikke læses'}}else{byId('rfidStatus').textContent='RFID: forbindelse afbrudt'}finish()}};
 x.onerror=x.ontimeout=function(){byId('rfidStatus').textContent='RFID: forbindelse afbrudt';finish()};x.send(null);
}
refreshRfid();
</script></div></body></html>)rawliteral";
}


void handleStateJson(){
 bool active=vehicleWasOnScale || currentWeight>=RFID_TRIGGER_WEIGHT;
 resolveCombination();
 float modelLoad=max(0.0f,currentWeight-combinedModelTare);
 float load=calculatedRealLoad();
 float display=primaryKnown?load:max(0.0f,currentWeight)*LOAD_SCALE_FACTOR;
 String j="{\"active\":"+String(active?"true":"false")+",\"scanning\":"+String(rfidSearching?"true":"false")+",\"stable\":"+String(stable?"true":"false")+",\"known\":"+String(primaryKnown?"true":"false")+",\"saved\":"+String(journeySaved?"true":"false")+",\"vehicle\":\""+jsonEscape(primaryName)+"\",\"owner\":\""+jsonEscape(primaryOwner)+"\",\"attachment\":\""+jsonEscape(attachmentNames)+"\",\"display\":"+String(display,0)+",\"modelLoad\":"+String(modelLoad,3)+",\"tare\":"+String(combinedRealTare,0)+",\"load\":"+String(load,0)+",\"gross\":"+String(calculatedRealGross(),0)+",\"from\":"+String(pickup)+",\"to\":"+String(dropoff)+",\"places\":[";
 for(int i=0;i<placeCount;i++){if(i)j+=",";j+="\""+jsonEscape(places[i])+"\"";}j+="]}";
 server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",j);
}
void handleClock(){if(!server.hasArg("epoch")){server.send(400,"text/plain","missing");return;}uint32_t e=(uint32_t)server.arg("epoch").toInt();if(e>1700000000UL){clockEpoch=e;clockAt=millis();server.send(200,"text/plain","OK");}else server.send(400,"text/plain","invalid");}
void handleSelect(){
 if(!vehicleWasOnScale){server.send(409,"text/plain","Ingen køretøj");return;}
 int n=server.arg("index").toInt();if(n<0||n>=placeCount){server.send(400,"text/plain","Ugyldigt sted");return;}
 String side=server.arg("side");
 if(side=="from"&&n!=dropoff)pickup=n;
 else if(side=="to"&&n!=pickup)dropoff=n;
 else {server.send(409,"text/plain","Samme sted kan ikke vælges");return;}
 server.send(200,"text/plain","OK");
}
void handlePlacesJson(){String j="[";for(int i=0;i<placeCount;i++){if(i)j+=",";j+="\""+jsonEscape(places[i])+"\"";}j+="]";server.send(200,"application/json",j);}
void handlePlaceSave(){
 String name=cleanText(server.arg("name"));name.replace("|"," ");if(name.length()<1||name.length()>35){server.send(400,"text/plain","Navn skal være 1-35 tegn");return;}
 int idx=server.hasArg("index")?server.arg("index").toInt():-1;
 for(int i=0;i<placeCount;i++)if(i!=idx&&places[i].equalsIgnoreCase(name)){server.send(409,"text/plain","Stedet findes");return;}
 if(idx>=0&&idx<placeCount)places[idx]=name;
 else {if(placeCount>=MAX_PLACES){server.send(409,"text/plain","Register fuldt");return;}places[placeCount++]=name;}
 savePlaces();server.send(200,"text/plain","OK");
}
void handlePlaceDelete(){
 int idx=server.arg("index").toInt();if(idx<0||idx>=placeCount){server.send(400,"text/plain","Ugyldigt");return;}
 if(vehicleWasOnScale){server.send(409,"text/plain","Vent til vægten er tom");return;}
 for(int i=idx;i<placeCount-1;i++)places[i]=places[i+1];placeCount--;savePlaces();pickup=dropoff=-1;server.send(200,"text/plain","OK");
}
String placesPage(){return R"V18PL(<!doctype html><html lang="da"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Pladsregister</title><style>body{background:#191919;color:white;font:18px Arial;padding:20px;max-width:720px;margin:auto}input,button{font:18px Arial;padding:13px;margin:5px;border-radius:8px}button{background:#444;color:white;border:1px solid #777}input{width:65%}li{padding:7px}</style><a href="/" style="color:white">← Brovægt</a><h1>Pladsregister</h1><p>Opret og ret steder. Knapperne dannes automatisk på vægtsiden.</p><input id="name" placeholder="Navn på plads"><button onclick="save()">Gem sted</button><div id="msg"></div><ol id="list"></ol><script>var edit=-1,places=[];function api(url,body,cb){var x=new XMLHttpRequest();x.open(body===null?'GET':'POST',url);if(body!==null)x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');x.onload=function(){cb(x.status,x.responseText)};x.send(body)}function load(){api('/places.json',null,function(s,t){if(s!=200)return;places=JSON.parse(t);var box=document.getElementById('list');box.innerHTML='';places.forEach(function(p,i){var li=document.createElement('li');li.appendChild(document.createTextNode(p+' '));var e=document.createElement('button');e.textContent='Redigér';e.onclick=function(){edit=i;document.getElementById('name').value=p};var d=document.createElement('button');d.textContent='Slet';d.onclick=function(){if(confirm('Slet '+p+'?'))api('/place/delete','index='+i,function(s,t){document.getElementById('msg').textContent=s==200?'Slettet':t;load()})};li.appendChild(e);li.appendChild(d);box.appendChild(li)})})}function save(){var n=document.getElementById('name').value.trim();if(!n)return;api('/place/save','index='+edit+'&name='+encodeURIComponent(n),function(s,t){document.getElementById('msg').textContent=s==200?'Gemt':t;if(s==200){edit=-1;document.getElementById('name').value='';load()}})}load();</script></html>)V18PL";}
String reportPage(){return R"V18RP(<!doctype html><html lang="da"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Dagsrapport</title><style>body{background:#191919;color:white;font:17px Arial;padding:18px;max-width:960px;margin:auto}a{color:#bfe4ff}table{border-collapse:collapse;width:100%}td,th{border-bottom:1px solid #555;padding:9px;text-align:left}button{padding:12px;background:#444;color:white;border:1px solid #888;border-radius:8px}</style><a href="/">← Brovægt</a><h1>Dagsrapport</h1><p>Datoer hentes fra iPad. Tilslut iPad til BROVAEGT før dagens første vejning.</p><label>Dato <input type="date" id="date"></label> <a id="csv" href="/report.csv">Hent CSV</a><p id="total"></p><table><thead><tr><th>Tid</th><th>Køretøj</th><th>Fra</th><th>Til</th><th>Last (kg)</th></tr></thead><tbody id="rows"></tbody></table><script>var date=document.getElementById('date');date.value=new Date().toLocaleDateString('sv-SE');function update(){var d=date.value;document.getElementById('csv').href='/report.csv?date='+encodeURIComponent(d);var x=new XMLHttpRequest();x.open('GET','/report.json?date='+encodeURIComponent(d));x.onload=function(){if(x.status!=200)return;var a=JSON.parse(x.responseText),sum=0,b=document.getElementById('rows');b.innerHTML='';a.forEach(function(r){sum+=r.load;var tr=document.createElement('tr');[r.time,r.vehicle,r.from,r.to,Math.round(r.load).toLocaleString('da-DK')].forEach(function(v){var td=document.createElement('td');td.textContent=v;tr.appendChild(td)});b.appendChild(tr)});document.getElementById('total').textContent=a.length+' transporter · '+Math.round(sum).toLocaleString('da-DK')+' kg samlet last'};x.send()}date.onchange=update;update();</script></html>)V18RP";}
void handleReportJson(){
 String day=server.arg("date"),j="[";bool first=true;
 if(logStorageReady){File f=LittleFS.open("/weighings.csv","r");if(f){while(f.available()){
 String line=f.readStringUntil('\n');line.trim();int p1=line.indexOf(';'),p2=line.indexOf(';',p1+1),p3=line.indexOf(';',p2+1),p4=line.indexOf(';',p3+1),p5=line.indexOf(';',p4+1);
 if(p1<0||p2<0||p3<0||p4<0||p5<0)continue;
 if(day.length()&&line.substring(0,p1)!=day)continue;
 if(!first)j+=",";first=false;
 j+="{\"time\":\""+jsonEscape(line.substring(p1+1,p2))+"\",\"vehicle\":\""+jsonEscape(line.substring(p2+1,p3))+"\",\"from\":\""+jsonEscape(line.substring(p3+1,p4))+"\",\"to\":\""+jsonEscape(line.substring(p4+1,p5))+"\",\"load\":"+String(line.substring(p5+1).toFloat(),0)+"}";
 }f.close();}}
 j+="]";server.send(200,"application/json",j);
}
void handleReportCsv(){
 String day=server.arg("date");
 server.sendHeader("Content-Disposition","attachment; filename=brovaegt.csv");
 server.setContentLength(CONTENT_LENGTH_UNKNOWN);server.send(200,"text/csv; charset=utf-8","");
 server.sendContent("Dato;Tid;Koeretoej;Afhentning;Aflaesning;Last_kg\r\n");
 if(logStorageReady){File f=LittleFS.open("/weighings.csv","r");if(f){while(f.available()){
 String line=f.readStringUntil('\n');line.trim();if(!line.length())continue;
 if(day.length()&&!line.startsWith(day+";"))continue;server.sendContent(line+"\r\n");
 }f.close();}}
 server.sendContent("");
}
void trySaveJourney(){
 if(journeySaved)return;
 // Diagnose only when a vehicle is on the scale; at most once every 3 seconds.
 static unsigned long lastLogDiagnostic=0;
 String reason="";
 if(!vehicleWasOnScale) return;
 if(rfidSearching)reason="RFID scanning still active";
 else if(!stable)reason="Weight not stable";
 else if(!primaryKnown)reason="Vehicle not recognized";
 else if(unknownTagCount>0)reason="Unknown RFID tag present";
 else if(pickup<0||dropoff<0)reason="Pickup or dropoff not selected";
 else if(pickup==dropoff)reason="Pickup and dropoff are identical";
 else if(!clockReady)reason="CLOCK NOT SYNCED - browser time missing";
 else if(!logStorageReady)reason="LITTLEFS NOT READY - storage unavailable";
 else if(calculatedRealLoad()<=0)reason="Calculated load is zero or negative";
 if(reason.length()){
   if(millis()-lastLogDiagnostic>=3000UL){
     Serial.print("SAVE BLOCKED: ");Serial.println(reason);
     lastLogDiagnostic=millis();
   }
   return;
 }
 float load=calculatedRealLoad();
 if(!appendLog(safeCsv(dateNow())+";"+safeCsv(timeNow())+";"+safeCsv(primaryName)+";"+safeCsv(places[pickup])+";"+safeCsv(places[dropoff])+";"+String(load,0)+"\n")){Serial.println("LOG WRITE FAILED");return;}
 journeySaved=true;Serial.println("TRANSPORT SAVED");
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
  for(int i=0;i<resultTagCount;i++){
    if(i>0) json+=",";
    json+="\""+jsonEscape(resultTags[i])+"\"";
  }
  json+="]";
  server.sendHeader("Cache-Control","no-store");
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

void startRfidSearch(bool manual){
  if(rfidSearching) return;
  rfidManualSearch=manual;
  rfidFinished=false;
  resultTagCount=0;
  for(int i=0;i<MAX_SCANNED_TAGS;i++) resultTags[i]="";
  if(!manual) clearScan();
  rfidSearching=true;
  rfidStopSent=false;
  lastRfidPoll=0;
  rfidPauseUntil=0;
  rfidSearchStarted=millis();
  resetRfidFrame();
  Serial.println("RFID MULTI-TAG SEARCH STARTED - 10 SECOND WINDOW");
}

void stopRfidSearch(bool announce){
  rfidSearching=false;
  rfidFinished=true;
  lastRfidPoll=0;
  rfidPauseUntil=0;
  rfidSearchStarted=0;
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
  if(rfidPauseUntil && (long)(now-rfidPauseUntil)<0) return;
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
  bool duplicate=false;
  for(int i=0;i<resultTagCount;i++) if(resultTags[i]==epc) duplicate=true;
  if(!duplicate && resultTagCount<MAX_SCANNED_TAGS){
    resultTags[resultTagCount++]=epc;
    if(!rfidManualSearch){
      scannedTags[scannedTagCount++]=epc;
      resolveCombination();
    }
    Serial.print("RFID TAG: "); Serial.println(epc);
  }
  rfidPauseUntil=millis()+RFID_AFTER_TAG_PAUSE;
}

void handleRfidInput(){
  if(rfidFrameLength && millis()-rfidFrameStarted>100) resetRfidFrame();
  // Bound UART work so continuous/noisy input cannot starve HTTP handling.
  int budget=128;
  while(budget-- > 0 && Serial2.available()>0){
    byte rfidByte=Serial2.read();
    if(!rfidSearching) continue;
    if(rfidFrameLength==0){ if(rfidByte==0xAA){ rfidFrame[rfidFrameLength++]=rfidByte; rfidFrameStarted=millis(); } continue; }
    // 0xAA is legal inside the payload; framing follows the declared length.
    if(rfidFrameLength>=RFID_MAX_FRAME_LENGTH){ resetRfidFrame(); continue; }
    rfidFrame[rfidFrameLength++]=rfidByte;
    if(rfidFrameLength>=5){
      size_t expectedLength=((size_t)rfidFrame[3]<<8 | rfidFrame[4])+7;
      if(expectedLength>RFID_MAX_FRAME_LENGTH) resetRfidFrame();
      else if(rfidFrameLength==expectedLength){
        if(rfidFrame[rfidFrameLength-1]==0xDD && rfidFrame[1]==0x02 && rfidFrame[2]==0x22 && rfidFrameLength>=12 && rfidFrameChecksumValid()) processRfidTagFrame();
        resetRfidFrame();
      } else if(rfidFrameLength>expectedLength) resetRfidFrame();
    }
  }
}

void updateVehicleCycle(){
  if(!rfidSearching && !vehicleWasOnScale && currentWeight>=RFID_TRIGGER_WEIGHT){
    vehicleWasOnScale=true;
    vehicleLeftTimer=0;
    startRfidSearch(false);
  }

  if(rfidSearching && millis()-rfidSearchStarted>=RFID_SCAN_DURATION){
    stopRfidSearch(true);
  }

  if(vehicleWasOnScale && currentWeight<VEHICLE_LEFT_WEIGHT){
    if(vehicleLeftTimer==0) vehicleLeftTimer=millis();
    if(millis()-vehicleLeftTimer>=VEHICLE_LEFT_DELAY){
      if(rfidSearching && !rfidManualSearch) stopRfidSearch(true);
      clearScan();
      pickup=-1;dropoff=-1;journeySaved=false;
      vehicleWasOnScale=false;
      vehicleLeftTimer=0;
      rfidStopSent=false;
      Serial.println("RFID READY - WAITING FOR 0.500 kg");
    }
  } else if(currentWeight>=VEHICLE_LEFT_WEIGHT){
    vehicleLeftTimer=0;
  }
}

void handleRfidStatus(){
  unsigned long elapsed=millis()-rfidSearchStarted;
  unsigned long remaining=rfidSearching && elapsed<RFID_SCAN_DURATION ? (RFID_SCAN_DURATION-elapsed+999)/1000 : 0;
  String json="{\"state\":\""+String(rfidSearching?"SCANNER":rfidFinished?"FÆRDIG":"KLAR")+"\",\"remaining\":"+String(remaining)+",\"tags\":[";
  for(int i=0;i<resultTagCount;i++){
    if(i) json+=",";
    int idx=findVehicleIndex(resultTags[i]);
    json+="{\"epc\":\""+jsonEscape(resultTags[i])+"\",\"known\":"+String(idx>=0?"true":"false")+",\"name\":\""+jsonEscape(idx>=0?vehicles[idx].name:String(""))+"\"}";
  }
  json+="]}";
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.send(200,"application/json; charset=utf-8",json);
}

void handleRfidTest(){
  if(rfidSearching){ server.send(409,"text/plain; charset=utf-8","RFID scanner allerede."); return; }
  startRfidSearch(true);
  handleRfidStatus();
}

// V1.7.4: Read-only R200 transmit-power diagnostic at startup.
// R200 protocol: 0xB7 returns two bytes in hundredths of dBm.

void readRfidFrequencyDiagnostics(){
  // Read-only R200 commands: 0x08 = region, 0xAA = current channel.
  const byte commands[2]={0x08,0xAA};
  int region=-1;
  int channel=-1;
  for(int commandIndex=0;commandIndex<2;commandIndex++){
    const byte cmd=commands[commandIndex];
    while(Serial2.available()) Serial2.read();
    const byte request[7]={0xAA,0x00,cmd,0x00,0x00,cmd,0xDD};
    Serial2.write(request,sizeof(request));
    byte frame[32];
    size_t used=0;
    bool found=false;
    unsigned long started=millis();
    while(millis()-started<1200 && !found){
      if(!Serial2.available()){ delay(1); continue; }
      byte b=Serial2.read();
      if(used==0 && b!=0xAA) continue;
      frame[used++]=b;
      if(used>=5){
        size_t total=(((size_t)frame[3]<<8)|frame[4])+7;
        if(total>sizeof(frame)){ used=0; continue; }
        if(used==total){
          byte sum=0;
          for(size_t i=1;i<used-2;i++) sum+=frame[i];
          if(frame[used-1]==0xDD && sum==frame[used-2] &&
             frame[1]==0x01 && frame[2]==cmd &&
             frame[3]==0x00 && frame[4]==0x01){
            if(cmd==0x08) region=frame[5];
            else channel=frame[5];
            found=true;
          }
          used=0;
        }
      }
    }
    if(!found){
      if(cmd==0x08) Serial.println("RFID REGION: no valid reply (unchanged)");
      else Serial.println("RFID CHANNEL: no valid reply (unchanged)");
    }
  }
  if(region>=0){
    Serial.print("RFID REGION CODE: 0x");
    if(region<16) Serial.print("0");
    Serial.print(region,HEX);
    if(region==0x03) Serial.println(" (EU 865-868 MHz)");
    else if(region==0x02) Serial.println(" (US 902-928 MHz)");
    else if(region==0x01) Serial.println(" (China 900 MHz)");
    else if(region==0x04) Serial.println(" (China 800 MHz)");
    else if(region==0x06) Serial.println(" (Korea)");
    else Serial.println(" (unknown region)");
  }
  if(channel>=0){
    Serial.print("RFID CHANNEL INDEX: ");
    Serial.println(channel);
    float mhz=0;
    if(region==0x03) mhz=865.1f+0.2f*channel;
    else if(region==0x02) mhz=902.25f+0.5f*channel;
    else if(region==0x01) mhz=920.125f+0.25f*channel;
    else if(region==0x04) mhz=840.125f+0.25f*channel;
    else if(region==0x06) mhz=917.1f+0.2f*channel;
    if(mhz>0){
      Serial.print("RFID CURRENT CHANNEL: ");
      Serial.print(mhz,3);
      Serial.println(" MHz (may change with frequency hopping)");
    }else Serial.println("RFID CHANNEL FREQUENCY: unknown until region is confirmed");
  }
  Serial.println("RFID FREQUENCY DIAGNOSTIC COMPLETE (read only)");
}

void readRfidTransmitPower(){
  const byte getPowerCmd[7]={0xAA,0x00,0xB7,0x00,0x00,0xB7,0xDD};
  while(Serial2.available()) Serial2.read();
  Serial2.write(getPowerCmd,sizeof(getPowerCmd));
  Serial2.flush();
  Serial.println("RFID TX POWER: requesting current setting...");
  byte frame[32];
  size_t used=0;
  unsigned long started=millis();
  while(millis()-started<1000){
    if(!Serial2.available()){ delay(1); continue; }
    byte b=Serial2.read();
    if(used==0){ if(b!=0xAA) continue; }
    frame[used++]=b;
    if(used>=5){
      size_t total=(((size_t)frame[3]<<8)|frame[4])+7;
      if(total>sizeof(frame)){ used=0; continue; }
      if(used==total){
        byte sum=0;
        for(size_t i=1;i<used-2;i++) sum+=frame[i];
        if(frame[used-1]==0xDD && sum==frame[used-2] &&
           frame[1]==0x01 && frame[2]==0xB7 && frame[3]==0 && frame[4]==2){
          unsigned int hundredths=((unsigned int)frame[5]<<8)|frame[6];
          Serial.print("RFID TX POWER: ");
          Serial.print(hundredths/100.0f,2);
          Serial.println(" dBm (read only)");
          return;
        }
        used=0;
      }
    }
  }
  Serial.println("RFID TX POWER: no valid reply (setting unchanged)");
}

void setup(){
  Serial.begin(115200);
  scale.begin(DOUT,SCK);
  delay(1000);
  tareScale();
  loadVehicles();
  loadPlaces();
  logStorageReady=LittleFS.begin(false);
  if(!logStorageReady)Serial.println("LittleFS unavailable - logging disabled");
  Serial.print("UNITS LOADED: "); Serial.println(vehicleCount);
  Serial2.begin(115200,SERIAL_8N1,RFID_RX_PIN,RFID_TX_PIN);
  readRfidTransmitPower();
  readRfidFrequencyDiagnostics();
  clearScan();
  Serial.println("RFID READY - WAITING FOR 0.500 kg");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);
  server.on("/",HTTP_GET,handleRoot);
  server.on("/state.json",HTTP_GET,handleStateJson);
  server.on("/clock",HTTP_POST,handleClock);
  server.on("/select",HTTP_POST,handleSelect);
  server.on("/places",HTTP_GET,[](){server.send(200,"text/html",placesPage());});
  server.on("/places.json",HTTP_GET,handlePlacesJson);
  server.on("/place/save",HTTP_POST,handlePlaceSave);
  server.on("/place/delete",HTTP_POST,handlePlaceDelete);
  server.on("/report",HTTP_GET,[](){server.send(200,"text/html",reportPage());});
  server.on("/report.json",HTTP_GET,handleReportJson);
  server.on("/report.csv",HTTP_GET,handleReportCsv);
  server.on("/status",HTTP_GET,handleStatus);
  server.on("/vehicles",HTTP_GET,handleVehiclesPage);
  server.on("/vehicles.json",HTTP_GET,handleVehiclesJson);
  server.on("/rfid-status",HTTP_GET,handleRfidStatus);
  server.on("/rfid-test",HTTP_POST,handleRfidTest);
  server.on("/current-tags",HTTP_GET,handleCurrentTags);
  server.on("/current-weight",HTTP_GET,handleCurrentWeight);
  server.on("/vehicle/save",HTTP_POST,handleVehicleSave);
  server.on("/vehicle/delete",HTTP_POST,handleVehicleDelete);
  server.begin();
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void loop(){
  updateVehicleCycle();
  trySaveJourney();
  handleRfidInput();
  handleRfidPolling();
  server.handleClient();
  updateVehicleCycle();
  // Preserve the existing 20-sample average without waiting for HX711.
  static long long rawSum=0;
  static int rawCount=0;
  if(!scale.is_ready()) return;
  // V1.7.6: Start RFID on two consecutive raw readings above 0.120 kg.
  // Keep the existing 20-sample average for displayed weight and stability.
  static int fastTriggerCount=0;
  long rawSample=scale.read();
  float fastWeight=(offset-rawSample)/calibrationFactor;
  if(!vehicleWasOnScale && !rfidSearching && fastWeight>=0.120f){
    if(++fastTriggerCount>=2){
      fastTriggerCount=0;
      vehicleWasOnScale=true;
      vehicleLeftTimer=0;
      startRfidSearch(false);
      Serial.println("RFID EARLY TRIGGER - RAW HX711");
    }
  } else {
    fastTriggerCount=0;
  }
  rawSum+=rawSample;
  if(++rawCount<20) return;
  long raw=rawSum/20;
  rawSum=0;
  rawCount=0;
  currentWeight=(offset-raw)/calibrationFactor;
  if(fabs(currentWeight-lastWeight)<0.005){
    if(stableTimer==0) stableTimer=millis();
    if(millis()-stableTimer>2000){ stable=true; lastStableWeight=currentWeight; }
  } else {
    stable=false;
    stableTimer=0;
  }
  updateVehicleCycle();
  trySaveJourney();
  lastWeight=currentWeight;
  server.handleClient();
}
