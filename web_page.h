#ifndef _WEB_PAGE_H_
#define _WEB_PAGE_H_

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WiFi Config</title>
  <style>
    body {
    background-color: #cccccc;
    font-family: Arial, Helvetica, Sans-Serif;
    Color: #000088; 
    }
    html {
    display: inline-block;
    margin: 0px auto;
    text-align: center;
    }
    h2 { font-size: 2.0rem; }
  .time { font-size: 1.2rem; }
  .cat { font-size: 1.6rem; }
  .opt { font-size: 1.4rem; }
  .txt { font-size: 1.2rem; }
  .cfg { text-align: left; font-size: 1.6rem; }
  </style>
</head>
<body>
  <h2>Pet Feeder</h2>
  %DATE%
  <p class="time">Uptime: %DD%days %HH%:%MM%:%SS%</p>n
  <p class="cfg"><input type="button" id="btn" onclick="hide_unhide()" value="%HIDE_BTN%"></p>
  <form class="cfg" id="configure" action="/config" onsubmit="do_submit();">
  <input type="hidden" id="cli_date" name="cli_date" value="0"/>
  <input type="hidden" id="tm_zone" name="tm_zone" value="0"/>
  <div class="cfg" id="wifi"%HIDE_WIFI_CFG%>
  <label class="cat">Select Network: </label><br>
  <select class="cat" name="ssid">
%WIFI_SSIDS%
  </select><br>
  <label class="cat">Password:</label><br>
  <input class="cat" type="text" name="pass" maxlength="32" size="12" value="%WIFI_PASS%"><br><br>
  </div>
  <label class="cat">Sleep Type:</label><br>
  <input type="radio" name="sleep" value="1"%SLEEP_1%><label class="opt">Deep Sleep</label><br>
  <input type="radio" name="sleep" value="2"%SLEEP_2%><label class="opt">Light Sleep</label><br>
  <input type="radio" name="sleep" value="3"%SLEEP_3%><label class="opt">Always On</label><br></br>
  <label class="opt">Feeder Calibration: <label class="opt" id="calV">%CAL%</label></label><br>
  <input type="range" name="cal" min="-100" max="100" style="width:380px" value="%CAL%" oninput="update_cal(this)"><br><br>
  <label class="opt">Dailly Schedule (HH:MM): </label><br>
  <div class="opt" id="d0" style="display:block">
  At:<input type="text" class="txt" name="tm0" id="tm0" value="%TM0%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr0" id="gr0" value="%GR0%" maxlength="3" size="1"> grams</label><br>
  </div>
  <div class="opt" id="d1" style="display:%TD1%">
  At:<input type="text" class="txt" name="tm1" id="tm1" value="%TM1%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr1" id="gr1" value="%GR1%" maxlength="3" size="1"> grams<br>
  </div>
  <div class="opt" id="d2" style="display:%TD2%">
  At:<input type="text" class="txt" name="tm2" id="tm2" value="%TM2%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr2" id="gr2" value="%GR2%" maxlength="3" size="1"> grams<br>
  </div>
  <div class="opt" id="d3" style="display:%TD3%">
  At:<input type="text" class="txt" name="tm3" id="tm3" value="%TM3%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr3" id="gr3" value="%GR3%" maxlength="3" size="1"> grams<br>
  </div>
  <div class="opt" id="d4" style="display:%TD4%">
  At:<input type="text" class="txt" name="tm4" id="tm4" value="%TM4%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr4" id="gr4" value="%GR4%" maxlength="3" size="1"> grams<br>
  </div>
  <div class="opt" id="d5" style="display:%TD5%">
  At:<input type="text" class="txt" name="tm5" id="tm5" value="%TM5%" maxlength="5" size="1" oninput="unhide_select(this)">
   drop:<input type="text" class="txt" name="gr5" id="gr5" value="%GR5%" maxlength="3" size="1"> grams<br>
  </div>
  <br>
  <input class="opt" type="submit" name="Submit">
  </form>
  <script>
  function do_submit() {
    var now = Date.now();
    var date = new Date();
    document.getElementById("cli_date").value = now;
    document.getElementById("tm_zone").value = date.getTimezoneOffset();
    return true;
  }
  function update_cal(obj) {
    var x = document.getElementById("calV");
    if (x)
      x.innerHTML = obj.value;
  }
  function hide_unhide() {
    var x = document.getElementById("wifi");
    var btn = document.getElementById("btn");
    if (x.style.display === "none") {
      x.style.display = "block";
      btn.value = "-";
    } else {
      x.style.display = "none";
      btn.value = "+";
    }
  }
  function unhide_select(obj) {
    var id = parseInt(obj.id.slice(-1))+1;
    var div = document.getElementById("d" + id);
    var sel = document.getElementById("tm" + id);
    while (div) {
      if (obj.value) {
        div.style.display = "block";
        break;
      } else {
        div.style.display = "none";
        sel.value = "";
        id++;
        div = document.getElementById("d" + id);
        sel = document.getElementById("tm" + id);
      }
    }
  }
  </script>
</body>
)rawliteral";
#endif // _WEB_PAGE_H_
