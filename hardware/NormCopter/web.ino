extern int maxm;

void handleCmd() 
{
  String out;
  int x;

  for (uint8_t i = 0; i < server.args(); i++)
  {
    //Serial.print(server.argName(i));
    //Serial.println(server.arg(i));

    if      (server.argName(i) == "CalA") 
      calibratingA = CALSTEPS;
    else if (server.argName(i) == "CalG") 
      calibratingG = CALSTEPS;
    else if (server.argName(i) == "KP") 
      Kp_rate = server.arg(i).toFloat();
    else if (server.argName(i) == "KI") 
      Ki_rate = server.arg(i).toFloat();
    else if (server.argName(i) == "KD") 
      Kd_rate = server.arg(i).toFloat();
    else if (server.argName(i) == "KPy") 
      Kp_yaw = server.arg(i).toFloat();
    else if (server.argName(i) == "KIy") 
      Ki_yaw = server.arg(i).toFloat();
    else if (server.argName(i) == "KDy") 
      Kd_yaw = server.arg(i).toFloat();
    else if (server.argName(i) == "KPa") 
      Kp_ang = server.arg(i).toFloat();
    else if (server.argName(i) == "KIa") 
      Ki_ang = server.arg(i).toFloat();
    else if (server.argName(i) == "KDa") 
      Kd_ang = server.arg(i).toFloat();
    else if (server.argName(i) == "KPay") 
      Kp_ayw = server.arg(i).toFloat();
    else if (server.argName(i) == "KIay") 
      Ki_ayw = server.arg(i).toFloat();
    else if (server.argName(i) == "KDay") 
      Kd_ayw = server.arg(i).toFloat();
    else if (server.argName(i) == "Kp_stable") 
      Kp_stable = server.arg(i).toFloat();
    else if (server.argName(i) == "Ki_stable") 
      Ki_stable = server.arg(i).toFloat();
    else if (server.argName(i) == "Kd_stable") 
      Kd_stable = server.arg(i).toFloat();
    else if (server.argName(i) == "MaxPow") 
      maxm = server.arg(i).toInt();
    else if (server.argName(i) == "CommF") 
      storeacc(); 
    else if (server.argName(i) == "motor1_correction") 
      motor_correction[0] = server.arg(i).toFloat();
    else if (server.argName(i) == "motor2_correction") 
      motor_correction[1] = server.arg(i).toFloat();
    else if (server.argName(i) == "motor3_correction") 
      motor_correction[2] = server.arg(i).toFloat();
    else if (server.argName(i) == "motor4_correction") 
      motor_correction[3] = server.arg(i).toFloat();
  }
  
  out =  "<!doctype html>\n";
  out += "<html><body><center>\n";  
  out += "<br>";
  out += "<p><b>ESP32 Copter PID</b></p>";

  if (armed) out += "Armed ";
  if (fmode) out += " Rate\n";
  out += "<br><br>";

  out += "<form method=\"post\">\n";
  out += "R/P&emsp; KP\n";
  out += "<input type=\"number\" name=\"KP\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kp_rate;
  out += "\">&emsp;\n";
  out += "KI\n";
  out += "<input type=\"number\" name=\"KI\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Ki_rate;
  out += "\">&emsp;\n";
  out += "KD\n";
  out += "<input type=\"number\" name=\"KD\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kd_rate;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";

  out += "<form method=\"post\">\n";
  out += "Yaw&emsp; KP\n";
  out += "<input type=\"number\" name=\"KPy\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kp_yaw;
  out += "\">&emsp;\n";
  out += "KI\n";
  out += "<input type=\"number\" name=\"KIy\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Ki_yaw;
  out += "\">&emsp;\n";
  out += "KD\n";
  out += "<input type=\"number\" name=\"KDy\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kd_yaw;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";


  out += "<form method=\"post\">\n";
  out += "Ang&emsp; KP\n";
  out += "<input type=\"number\" name=\"KPa\" step=\"0.1\" style=\"width:4em\" value=\"";
  out += Kp_ang;
  out += "\">&emsp;\n";
  out += "KI\n";
  out += "<input type=\"number\" name=\"KIa\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Ki_ang;
  out += "\">&emsp;\n";
  out += "KD\n";
  out += "<input type=\"number\" name=\"KDa\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kd_ang;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";

  out += "<form method=\"post\">\n";
  out += "A-Y&emsp; KP\n";
  out += "<input type=\"number\" name=\"KPay\" step=\"0.1\" style=\"width:4em\" value=\"";
  out += Kp_ayw;
  out += "\">&emsp;\n";
  out += "KI\n";
  out += "<input type=\"number\" name=\"KIay\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Ki_ayw;
  out += "\">&emsp;\n";
  out += "KD\n";
  out += "<input type=\"number\" name=\"KDay\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kd_ayw;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";

  out += "<form method=\"post\">\n";
  out += "Stable&emsp; KP\n";
  out += "<input type=\"number\" name=\"Kp_stable\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kp_stable;
  out += "\">&emsp;\n";
  out += "KI\n";
  out += "<input type=\"number\" name=\"Ki_stable\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Ki_stable;
  out += "\">&emsp;\n";
  out += "KD\n";
  out += "<input type=\"number\" name=\"Kd_stable\" step=\"0.01\" style=\"width:4em\" value=\"";
  out += Kd_stable;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";
  
  out += "<form method=\"post\">\n";
  out += "MaxPower \n";
  out += "<input type=\"number\" name=\"MaxPow\" step=\"10\" style=\"width:5em\" value=\"";
  out += maxm;
  out += "\">&emsp;\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";
  out += "<br>";
  
  out += "<button onclick=\"window.location.href='?CalA=\';\">ACC Calib</button>&emsp;\n";
  out += "<button onclick=\"window.location.href='?CalG=\';\">GYRO Calib</button>&emsp;\n";
  out += "<button onclick=\"window.location.href='?CommF1=\';\">WriteACC</button>&emsp;\n";
  out += "<br><br>";

  out += "<div style='width:400px; height:400px; position:relative; margin:20px auto;'>\n";
  
  // 左上のモーター (motor4)
  out += "<div style='position:absolute; left:50px; top:50px;'>\n";
  out += "<form method='post'>\n";
  out += "M4 (LeftTop)<br>\n";
  out += "<input type='number' name='motor4_correction' step='0.01' style='width:4em' value='" + String(motor_correction[3]) + "'><br>\n";
  out += "<input type='submit' style='margin-top:5px;'>\n";
  out += "</form></div>\n";
  
  // 右上のモーター (motor2)
  out += "<div style='position:absolute; right:50px; top:50px;'>\n";
  out += "<form method='post'>\n";
  out += "M2 (RightTop)<br>\n";
  out += "<input type='number' name='motor2_correction' step='0.01' style='width:4em' value='" + String(motor_correction[1]) + "'><br>\n";
  out += "<input type='submit' style='margin-top:5px;'>\n";
  out += "</form></div>\n";
  
  // 左下のモーター (motor3)
  out += "<div style='position:absolute; left:50px; bottom:50px;'>\n";
  out += "<form method='post'>\n";
  out += "M3 (LeftBack)<br>\n";
  out += "<input type='number' name='motor3_correction' step='0.01' style='width:4em' value='" + String(motor_correction[2]) + "'><br>\n";
  out += "<input type='submit' style='margin-top:5px;'>\n";
  out += "</form></div>\n";
  
  // 右下のモーター (motor1)
  out += "<div style='position:absolute; right:50px; bottom:50px;'>\n";
  out += "<form method='post'>\n";
  out += "M1 (RightBack)<br>\n";
  out += "<input type='number' name='motor1_correction' step='0.01' style='width:4em' value='" + String(motor_correction[0]) + "'><br>\n";
  out += "<input type='submit' style='margin-top:5px;'>\n";
  out += "</form></div>\n";
  
  // ×型の線
  out += "<div style='position:absolute; left:50%; top:50%; transform:translate(-50%, -50%);'>\n";
  out += "<div style='width:200px; height:4px; background:#666; transform:rotate(45deg);'></div>\n";
  out += "<div style='width:200px; height:4px; background:#666; transform:rotate(-45deg);'></div>\n";
  out += "</div>\n";
  
  out += "</div>\n";

  out += "</center></html>";
  server.send(200, "text/html", out);    
}
