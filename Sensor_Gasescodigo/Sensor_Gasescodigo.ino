#include <LiquidCrystal.h>

//Ligacoes de Hardware
#define Buzzer 10
#define MQ2_Analog 0
#define MQ2_Digital 2
#define SensorLevel 500
#define LCD_RS 9
#define LCD_E 8
#define LCD_D4 3
#define LCD_D5 4
#define LCD_D6 5
#define LCD_D7 6

int digital_read = 0;
int analog_read = 0;

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);//RS E D4 D5 D6 D7

void setup()
{
  // Configura os IOs
  pinMode(MQ2_Digital, INPUT);
  pinMode(Buzzer, OUTPUT);
  // Inicializa a serial
  Serial.begin(9600);
  //Inicializa o LCD
 lcd.begin(16, 2); 
}

void loop()
{
  // Le os dados do sensor MQ2
  digital_read = digitalRead(MQ2_Digital);
  analog_read = analogRead(MQ2_Analog);
  

  //apresenta as leituras na porta serial
  Serial.println("Saida Digital:");
  Serial.println(digital_read);
  Serial.println(" Saida Analogica:");
  Serial.println(analog_read);

  if (analog_read > SensorLevel)
  {
    // Apresenta os dados no LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ar contaminado!");
    // Aciona o Buzzer
    digitalWrite(Buzzer, HIGH);

  }
  else
  {
     // Apresenta os dados no LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ar normal");
    // Desliga o buzzer e o led vermelho
    digitalWrite(Buzzer, LOW);
  }
  
  delay(2000);
}

#include <191.7.184.2>                // Biblioteca para conexão Wi-Fi
#include <PubSubClient.h>         // Biblioteca para MQTT

// Defina as credenciais de Wi-Fi
const char* ssid = "2804:1468:8813:f100:bdc4:1fd2:e7cf:a51c";      // Substitua pelo nome da sua rede Wi-Fi
const char* password = "Isabeli10"; // Substitua pela sua senha Wi-Fi

// Configurações do Broker MQTT
const char* mqtt_server = "broker.mqtt.com"; // Substitua pelo seu broker MQTT
const int mqtt_port = 1883;                  // Porta do broker MQTT
const char* mqtt_user = "seu_usuario";       // Substitua com seu nome de usuário MQTT
const char* mqtt_pass = "sua_senha";         // Substitua com sua senha MQTT

// Definir o pino do sensor MQ-2 (analógico)
int sensorPin = A0;  // O sensor MQ-2 será conectado a um pino analógico (A0)

// Inicializar o cliente Wi-Fi e o cliente MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Função de setup
void setup() {
  Serial.begin(115200);            // Inicializa o Monitor Serial
  WiFi.begin(ssid, password);      // Conecta ao Wi-Fi

  // Aguarda conexão com a rede Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao Wi-Fi...");
  }
  Serial.println("Wi-Fi Conectado!");

  // Configura o cliente MQTT
  client.setServer(mqtt_server, mqtt_port);  // Define o servidor MQTT
  connectToMQTT();  // Tenta conectar ao Broker MQTT
}

// Função de loop
void loop() {
  if (!client.connected()) {
    connectToMQTT();  // Tenta reconectar ao Broker MQTT, caso a conexão tenha caído
  }

  // Lê o valor do sensor de gás (MQ-2)
  int gasValue = analogRead(sensorPin);
  Serial.print("Valor do Gás: ");
  Serial.println(gasValue);  // Imprime o valor no Monitor Serial

  // Envia a leitura para o Broker MQTT no tópico "casa/sala/gas"
  char message[50];
  sprintf(message, "Valor do gás: %d", gasValue);  // Cria a mensagem
  client.publish("casa/sala/gas", message);  // Publica no tópico

  client.loop();  // Mantém a conexão com o broker
  delay(5000);    // Aguarda 5 segundos antes de repetir
}

// Função para conectar ao Broker MQTT
void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando ao Broker MQTT...");
    if (client.connect("ArduinoClient", mqtt_user, mqtt_pass)) {
      Serial.println("Conectado ao Broker!");
    } else {
      Serial.print("Falha. Código de erro: ");
      Serial.println(client.state());
      delay(5000);  // Tenta novamente após 5 segundos
    }
  }
}
