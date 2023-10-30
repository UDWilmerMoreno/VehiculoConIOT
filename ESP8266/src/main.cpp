#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

int contconexion = 0;
const char *ssid = "MotoG";
const char *password = "ras123456";
char SERVER[50] = "postman.cloudmqtt.com";
int SERVERPORT = 13313;
String USERNAME = "aplicada";
char PASSWORD[50] = "0123456789";

unsigned long previousMillis = 0;
char charColor[15];
String strColor;
String strColorUltimo;

char PLACA[50];
char valueStr[15];
char valueStr1[15];
//String strtemp = "";
//char TEMPERATURA[50];
char MOVIMIENTO[50];
char COLOR[50];

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length)
{
  char PAYLOAD[5] = "    ";
  Serial.print("Mensaje Recibido: [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++)
  {
    PAYLOAD[i] = (char)payload[i];
  }

  Serial.println(PAYLOAD);

  if (String(topic) == String(MOVIMIENTO))
  {
    if (payload[0] == 'W')
    {
      Serial.println('w');
    }
    if (payload[0] == 'S')
    {
      Serial.println('s');
    }
    if (payload[0] == 'D')
    {
      Serial.println('d');
    }
    if (payload[0] == 'A')
    {
      Serial.println('a');
    }
    if (payload[0] == 'Z')
    {
      Serial.println('z');
    }
    if (payload[0] == 'X')
    {
      Serial.println('x');
    }
    if (payload[0] == 'C')
    {
      Serial.println('c');
    }
    if (payload[0] == 'V')
    {
      Serial.println('v');
    }
    if (payload[0] == 'B')
    {
      Serial.println('b');
    }
    if (payload[0] == 'N')
    {
      Serial.println('n');
    }
    if (payload[0] == 'M')
    {
      Serial.println('m');
    }
    if (payload[0] == 'L')
    {
      Serial.println('l');
    }
    if (payload[0] == 'O')
    {
      Serial.println('o');
    }
    if (payload[0] == 'P')
    {
      Serial.println('p');
    }
    if (payload[0] == 'U')
    {
      Serial.println('u');
    }
    if (payload[0] == 'I')
    {
      Serial.println('i');
    }
  }
}

void reconnect()
{
  uint8_t retries = 3;
  while (!client.connected())
  {
    Serial.print("Intentando conexion MQTT...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    USERNAME.toCharArray(PLACA, 50);
    if (client.connect("", PLACA, PASSWORD))
    {
      Serial.println("conectado");
      client.subscribe(MOVIMIENTO);
    }
    else
    {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intenta nuevamente en 5 segundos");
      delay(5000);
    }
    retries--;
    if (retries == 0)
    {
      while (1)
        ;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");

  while (!Serial)
  {
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED and contconexion < 50)
  {
    ++contconexion;
    delay(500);
    Serial.print(".");
  }

  if (contconexion < 50)
  {
    Serial.println("");
    Serial.println("WiFi conectado");
  }
  else
  {
    Serial.println("");
    Serial.println("Error de conexion");
  }

  client.setServer(SERVER, SERVERPORT);
  client.setCallback(callback);

  String movimiento = "/" + USERNAME + "/" + "Movimiento";
  movimiento.toCharArray(MOVIMIENTO, 50);
  String color = "/" + USERNAME + "/" + "Color";
  color.toCharArray(COLOR, 50);
  //String temperatura = "/" + USERNAME + "/" + "temperatura";
  //temperatura.toCharArray(TEMPERATURA, 50);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }

  client.loop();
  //unsigned long currentMillis = millis();

  if (Serial.available() > 0)
  {
    char dato = Serial.read();
    if (dato == 'f')
    {
      strColor = "ROJO";
    }
    else if (dato == 'g')
    {
      strColor = "VERDE";
    }
    else if (dato == 'h')
    {
      strColor = "AZUL";
    }
    else if (dato == 'j')
    {
      strColor = "SIN COLOR";
    }
    else
    {
      strColor = "SIN COLOR";
    }
  }

  if (strColor != strColorUltimo)
  {
    strColorUltimo = strColor;
    strColor.toCharArray(valueStr, 15);
    Serial.println("Enviando: [" + String(COLOR) + "] " + strColor);
    client.publish(COLOR, valueStr);
  }
  /*if (currentMillis - previousMillis >= 10000)
    { //envia la temperatura cada 10 segundos
        previousMillis = currentMillis;
        int analog = analogRead(17);
        float temp = analog * 0.322265625;
        strtemp = String(temp, 1); //1 decimal
        strtemp.toCharArray(valueStr, 15);
        Serial.println("Enviando: [" + String(TEMPERATURA) + "] " + strtemp);
        client.publish(TEMPERATURA, valueStr);
    }*/
}