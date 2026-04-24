void processCmdRequest() {
  if(!server.hasArg("c")) return returnFail("BAD ARGS");
  String cmd = server.arg("c");

  int robotCmd = findValidRobotCmd(cmd.c_str());

  if(robotCmd != -1) {
    IRbutton irbutton = buttonCmds[robotCmd];
    Serial.print("[HTTP] Got valid robot command: ");
    Serial.println(irbutton.name);
    
    server.send(200, "text/plain", "Robot "+String(irbutton.name));

    SendIRCode(irbutton);
  }

  else {
    returnFail("INVALID COMMAND");
  }
  
}

void returnFail(String msg) {
  server.send(200, "text/plain", msg);
}

void setupHTTP() {
//  MDNS.begin(mDNSname);
  
  httpUpdater.setup(&server, update_path, update_username, update_password);
  
  server.on("/", [](){
    server.send_P(200, "text/html", PAGE_index);
  });
  server.on("/version", [](){
  server.send(200, "text/plain", FIRMWARE_VERSION);
});
  server.on("/cmd", HTTP_GET, processCmdRequest);
  
  server.on("/battery", HTTP_GET, []() {
    float voltage = calcBattery(false);
    float percent = calcBattery(true);
    String json = "{\"voltage\":" + String(voltage, 2) + ",\"percent\":" + String(percent, 0) + "}";
    server.send(200, "application/json", json);
  });
  server.on("/status", HTTP_GET, []() {
  String statusText = getStatusText();
  float voltage = calcBattery(false);
  float percent = calcBattery(true);
  int adcRaw = analogRead(A0);
  
  String json = "{";
  json += "\"status\":\"" + statusText + "\",";
  json += "\"Battery_Voltage\":" + String(voltage, 2) + ",";
  json += "\"Battery_Level\":" + String(percent, 0) + ",";
  json += "\"ADC_Raw\":" + String(adcRaw);
  json += "}";
  
  server.send(200, "application/json", json);
});
  
  server.begin();
//  MDNS.addService("http", "tcp", 80);
  Serial.printf("\n[HTTP] HTTPUpdateServer ready! Open http://%s%s in your browser and login with username '%s' and password '%s'\n", WiFi.localIP().toString().c_str(), update_path, update_username, update_password);
  Serial.println("");
}

//void setupHTTP() {
//  MDNS.begin(mDNSname);
  
  //httpUpdater.setup(&server, update_path, update_username, update_password);
  //server.begin();

//  MDNS.addService("http", "tcp", 80);
  //Serial.printf("\n[HTTP] HTTPUpdateServer ready! Open http://%s%s in your browser and login with username '%s' and password '%s'\n", WiFi.localIP().toString().c_str(), update_path, update_username, update_password);
  //Serial.println("");
  
  //server.on("/", [](){
    //server.send_P(200, "text/html", PAGE_index);
  //});
  
  //server.on("/cmd", HTTP_GET, processCmdRequest);
//}








//
//void HTTPRobotAction(IRbutton irbutton){
//  //SendIRCode(irbutton);
//  
//  Serial.print("[HTTP] Got robot request: ");
//  Serial.println(irbutton.name);
//  
//  server.send(200, "text/plain", "Robot "+String(irbutton.name));
//
//  char command[20];
//  sprintf(command, "robot;%s", irbutton.name);
//  mqtt.publish(outTopic, command);
//}
//    
//void setupHTTPRobotCmd() {
//  for(int c = 0; c < sizeof(buttonCmds)/sizeof(buttonCmds[0]); c++) {
//    
//    char requesturl[20];
//    sprintf(requesturl, "/cmd=%s", buttonCmds[c].name);
//
//    //handle http request
//    server.on(requesturl, std::bind(&HTTPRobotAction, buttonCmds[c]));
//  }
//}
