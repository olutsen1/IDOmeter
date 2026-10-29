// =====================================================================
//  IDometer — Wi-Fi без перекомпиляции
//
//  * Прибор помнит до 5 сетей (во встроенной памяти, переживает выключение)
//    и сам подключается к той, что рядом.
//  * Режим настройки: прибор поднимает свою сеть «IDometer-setup».
//    Подключитесь к ней телефоном — откроется страница выбора сети
//    (если не открылась сама, наберите в браузере 192.168.4.1).
//    Включается сам, если 30 с нет связи, или если держать правую кнопку
//    при включении прибора.
//  * Монитор порта (115200): команды wifi, list, forget N, setup, status, help.
// =====================================================================
#pragma once
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#define MAX_NETS 5
#define AP_NAME  "IDometer-setup"

struct Net { String ssid, pass; };
static Net  nets[MAX_NETS];
static int  netCount = 0;

volatile bool stApOn = false;
char          stSsid[33] = "";
volatile bool forceApAtBoot = false;

static Preferences prefs;
static WebServer   web(80);
static DNSServer   dns;
static WiFiMulti*  multi = nullptr;
static bool        multiDirty = true;
static bool        tryNow = false;
static String      scanList[20];
static int         scanCount = 0;

// ---------------------------------------------------------------- память
static void netsLoad() {
  prefs.begin("wifi", true);
  netCount = min((int)prefs.getUChar("n", 0), MAX_NETS);
  for (int i = 0; i < netCount; i++) {
    nets[i].ssid = prefs.getString(("s" + String(i)).c_str(), "");
    nets[i].pass = prefs.getString(("p" + String(i)).c_str(), "");
  }
  prefs.end();
}

static void netsSave() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.putUChar("n", netCount);
  for (int i = 0; i < netCount; i++) {
    prefs.putString(("s" + String(i)).c_str(), nets[i].ssid);
    prefs.putString(("p" + String(i)).c_str(), nets[i].pass);
  }
  prefs.end();
  multiDirty = true;
}

static void netsForget(int idx) {
  if (idx < 0 || idx >= netCount) return;
  for (int i = idx; i < netCount - 1; i++) nets[i] = nets[i + 1];
  netCount--;
  netsSave();
}

// новая (или изменённая) сеть встаёт первой; самая старая вытесняется
static void netsAdd(const String& ssid, const String& pass) {
  if (!ssid.length()) return;
  for (int i = 0; i < netCount; i++)
    if (nets[i].ssid == ssid) { netsForget(i); break; }
  for (int i = min(netCount, MAX_NETS - 1); i > 0; i--) nets[i] = nets[i - 1];
  nets[0] = {ssid, pass};
  netCount = min(netCount + 1, MAX_NETS);
  netsSave();
  tryNow = true;
}

// ---------------------------------------------------------------- страница
static String esc(const String& s) {
  String o;
  for (char c : s) {
    if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;"; else if (c == '"') o += "&quot;";
    else o += c;
  }
  return o;
}

static void doScan() {
  int n = WiFi.scanNetworks();
  scanCount = 0;
  for (int i = 0; i < n && scanCount < 20; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;
    bool dup = false;
    for (int j = 0; j < scanCount; j++) if (scanList[j] == s) dup = true;
    if (!dup) scanList[scanCount++] = s;
  }
  WiFi.scanDelete();
}

static const char* PAGE_HEAD =
  "<!doctype html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>IDometer</title><style>"
  "body{font-family:sans-serif;max-width:420px;margin:16px auto;padding:0 12px;color:#222}"
  "h1{font-size:22px}select,input,button{width:100%;font-size:17px;padding:10px;margin:6px 0;box-sizing:border-box}"
  "button{background:#1565c0;color:#fff;border:0;border-radius:6px}"
  ".net{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #ddd}"
  "a{color:#1565c0}.ok{color:#2e7d32}.no{color:#c62828}</style></head><body>";

static void pageRoot() {
  String h = PAGE_HEAD;
  h += "<h1>IDometer — сеть Wi-Fi</h1>";
  if (WiFi.status() == WL_CONNECTED)
    h += "<p class='ok'>Подключён к «" + esc(WiFi.SSID()) + "»</p>";
  else
    h += "<p class='no'>Сейчас не подключён к сети</p>";
  h += "<form method='post' action='/save'><label>Сеть рядом:</label><select name='ssid'>";
  for (int i = 0; i < scanCount; i++)
    h += "<option>" + esc(scanList[i]) + "</option>";
  h += "</select><label>или впишите имя сети вручную:</label>"
       "<input name='ssid_text' placeholder='(необязательно)'>"
       "<label>Пароль:</label><input name='pass' type='password'>"
       "<button>Сохранить и подключиться</button></form>"
       "<p><a href='/scan'>Обновить список сетей</a></p>"
       "<h3>Сохранённые сети</h3>";
  if (!netCount) h += "<p>пока нет</p>";
  for (int i = 0; i < netCount; i++)
    h += "<div class='net'><span>" + esc(nets[i].ssid) +
         "</span><a href='/forget?i=" + String(i) + "'>забыть</a></div>";
  h += "<p style='color:#777;font-size:13px'>Прибор помнит до 5 сетей и сам выбирает ту, что рядом.</p></body></html>";
  web.send(200, "text/html; charset=utf-8", h);
}

static void pageSave() {
  String ssid = web.arg("ssid_text");
  ssid.trim();
  if (!ssid.length()) ssid = web.arg("ssid");
  netsAdd(ssid, web.arg("pass"));
  String h = PAGE_HEAD;
  h += "<h1>Сохранено</h1><p>Прибор подключается к «" + esc(ssid) + "».</p>"
       "<p>Телефон сейчас отключится от сети IDometer-setup — это нормально. "
       "Смотрите на экран прибора: должно появиться <b>WiFi: OK</b>.</p>"
       "<p>Если через минуту прибор снова покажет режим настройки — "
       "скорее всего, неверный пароль.</p></body></html>";
  web.send(200, "text/html; charset=utf-8", h);
}

static void redirectRoot() {
  web.sendHeader("Location", "http://192.168.4.1/", true);
  web.send(302, "text/plain", "");
}

static void apStart() {
  if (stApOn) return;
  WiFi.mode(WIFI_AP_STA);
  doScan();
  WiFi.softAP(AP_NAME);
  dns.start(53, "*", WiFi.softAPIP());
  web.on("/", HTTP_GET, pageRoot);
  web.on("/save", HTTP_POST, pageSave);
  web.on("/scan", HTTP_GET, [] { doScan(); redirectRoot(); });
  web.on("/forget", HTTP_GET, [] { netsForget(web.arg("i").toInt()); redirectRoot(); });
  web.onNotFound(redirectRoot);     // любой адрес ведёт на страницу настройки
  web.begin();
  stApOn = true;
  Serial.println("Режим настройки: сеть «" AP_NAME "», страница 192.168.4.1");
}

static void apStop() {
  if (!stApOn) return;
  web.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  stApOn = false;
  Serial.println("Режим настройки выключен");
}

// ---------------------------------------------------------------- монитор порта
static void printNets() {
  Serial.printf("Сохранённых сетей: %d\n", netCount);
  for (int i = 0; i < netCount; i++) Serial.printf("  %d: %s\n", i + 1, nets[i].ssid.c_str());
}

static void printHelp() {
  Serial.println(
    "Команды:\n"
    "  wifi      — добавить сеть (спросит имя и пароль)\n"
    "  list      — сохранённые сети\n"
    "  forget N  — забыть сеть номер N\n"
    "  setup     — включить режим настройки с телефона\n"
    "  status    — состояние прибора\n"
    "  help      — эта подсказка");
}

void printStatus();   // в основном файле

static void serialPoll() {
  static String buf;
  static uint32_t lastChar = 0;
  static int step = 0;          // 0 — команда, 1 — ждём имя сети, 2 — ждём пароль
  static String newSsid;

  bool lineDone = false;
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\r' || c == '\n') { if (buf.length()) { lineDone = true; break; } }
    else buf += c;
  }
  // монитор порта без перевода строки: строка закончилась, если 1 с тишины
  if (!lineDone && buf.length() && millis() - lastChar > 1000) lineDone = true;
  if (!lineDone) return;

  String s = buf;
  buf = "";
  if (step == 1) {
    newSsid = s;
    Serial.println("Пароль (пустая строка — сеть без пароля, тогда отправьте пробел):");
    step = 2;
    return;
  }
  if (step == 2) {
    s.trim();
    netsAdd(newSsid, s);
    Serial.printf("Сохранено: «%s». Подключаюсь…\n", newSsid.c_str());
    step = 0;
    return;
  }
  s.trim();
  if (s == "wifi") { Serial.println("Имя сети (SSID), точно как в списке сетей:"); step = 1; }
  else if (s == "list") printNets();
  else if (s.startsWith("forget")) {
    int n = s.substring(6).toInt();
    if (n >= 1 && n <= netCount) { Serial.printf("Забыта: %s\n", nets[n - 1].ssid.c_str()); netsForget(n - 1); }
    else Serial.println("Укажите номер из list, например: forget 2");
  }
  else if (s == "setup") apStart();
  else if (s == "status") printStatus();
  else printHelp();
}

// ---------------------------------------------------------------- задача Wi-Fi
void wifiTask(void*) {
  netsLoad();
  String def = WIFI_SSID;
  if (netCount == 0 && def.length() && def != "имя_вашей_сети") netsAdd(def, WIFI_PASSWORD);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);         // энергосбережение Wi-Fi даёт задержки до сотен мс
  if (netCount == 0 || forceApAtBoot) apStart();

  uint32_t lostSince = millis(), lastTry = 0, apIdleSince = millis();
  printHelp();
  printNets();

  for (;;) {
    serialPoll();
    if (stApOn) { dns.processNextRequest(); web.handleClient(); }

    if (WiFi.status() == WL_CONNECTED) {
      lostSince = millis();
      strlcpy(stSsid, WiFi.SSID().c_str(), sizeof stSsid);
      // связь есть и никто не настраивает — режим настройки больше не нужен
      if (stApOn) {
        if (WiFi.softAPgetStationNum() > 0) apIdleSince = millis();
        else if (millis() - apIdleSince > 60000) apStop();
      }
    } else {
      stSsid[0] = 0;
      if (!stApOn && millis() - lostSince > 30000) { apStart(); apIdleSince = millis(); }
      // пока телефон подключён к настройке, сеть не ищем: поиск сбивает ему связь
      bool apBusy = stApOn && WiFi.softAPgetStationNum() > 0;
      uint32_t period = stApOn ? 15000 : 3000;
      if (netCount && (tryNow || (!apBusy && millis() - lastTry > period))) {
        if (tryNow) delay(1500);     // дать странице «Сохранено» дойти до телефона
        tryNow = false;
        if (multiDirty || !multi) {
          delete multi;
          multi = new WiFiMulti();
          for (int i = 0; i < netCount; i++) multi->addAP(nets[i].ssid.c_str(), nets[i].pass.c_str());
          multiDirty = false;
        }
        multi->run(8000);
        lastTry = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
