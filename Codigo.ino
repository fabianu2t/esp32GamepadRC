#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>
#include "Adafruit_MCP23X17.h"
#include <Wire.h>

/* Se penso este codigo para una pala mecanica 
    utiliza esp32, MCP23X17, 4 DRV8833, 6 motores, 4 sensores y un control de consola
    
*/

// Motores de giro
#define motorGiro1 14
#define motorGiro2 15
// Motores del braso 
#define motorBraso1 10
#define motorBraso2  11
#define motorCodo1 3
#define motorCodo2 2

//Motor pala 
#define motorPala1 9
#define motorPala2 8
// Motores de Movimiento
#define leftMotor0 7
#define leftMotor1 6
#define rightMotor0 4
#define rightMotor1 5
//pin de sonsores de carrera
#define iBrazo 35
#define fBrazo 34

#define iPala 33
#define fPala 25


ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Adafruit_MCP23X17 mcp;

int aceleradorD = 0;
int aceleradorI = 0;
int aceleradorG = 0;
int movimientoBraso = 0;
int movimientoCodo = 0;
int movimientopala = 0;


// Función para mapear los valores del joystick a PWM
int joystickAPWM(int axisValue) {
  // El valor del joystick puede ir de -512 a 512, lo mapeamos a 0-255
  int pwmValue = map(axisValue, -512, 512, -250, 250);
  return constrain(pwmValue, -250, 250);  // Aseguramos que esté entre 0 y 255
}


// Función para mapear los valores del acalerador y freno a a PWM
int aceleradorAPWM(int axisValue) {
  // El valor del joystick puede ir de -0 a 1023, lo mapeamos a 0-255
  int pwmValue = map(axisValue, -512, 512, -250, 250);
  return constrain(pwmValue, -250, 250);  // Aseguramos que esté entre 0 y 255
}


void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Control  conectado, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Control modelo: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("Alerta!: Control conerctado, pero sin motores asignados\n");
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Alerta!: Control %d desconectado \n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("Alerta!: Control desconectado\n");
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Alerta!: Control soportado\n");
      }
    }
  }
}
void processGamepad(ControllerPtr ctl) {
  //Boom


    /*  
    control de el Motor Derecho
    Función para generar PWM por software en el MCP23017
    Controlar la velocidad del motor derecho según el joystick derecho
    */
    int rightMotorSpeed = aceleradorAPWM(ctl->throttle());
    if (rightMotorSpeed > 30) { 
      //movimientoCodo = potenciaCodo;
      aceleradorD = rightMotorSpeed;
      Serial.printf("Potencia de codo = %d\n", rightMotorSpeed);
      }else if(ctl->r1() == 1) {
      aceleradorD = -250;
      }else {  // Detener
      
     aceleradorD = 0;
      }


  /*control de el Motor Izquierdo
  Función para generar PWM por software en el MCP23017
  Controlar la velocidad del motor derecho según el joystick 
  */
  int leftMotorSpeed = aceleradorAPWM(ctl->brake());
    if (leftMotorSpeed > 30) { 
      //movimientoCodo = potenciaCodo;
      aceleradorI = leftMotorSpeed;
      Serial.printf("Potencia de codo = %d\n", leftMotorSpeed);
      }else if(ctl->l1() == 1) {
    aceleradorI = -250;
    }else {  // Detener
     aceleradorI = 0;
    }



  /*control de el Motor de Giro
  Función para generar PWM por software en el MCP23017
  Controlar la velocidad del motor derecho según el joystick derecho
  */

  int potenciaDeGiro = joystickAPWM(ctl->axisX());
    if (potenciaDeGiro > 30) { 
      aceleradorG = potenciaDeGiro;
      Serial.printf("Potencia de giro = %d\n", potenciaDeGiro);
      
    }else if (potenciaDeGiro < -30) { 
      aceleradorG = potenciaDeGiro;
      Serial.printf("Potencia de giro = %d\n", potenciaDeGiro);
      
    } else {  // Detener
      
     aceleradorG = 0;
    }

 
  /*control de el Motor de Braso
  Función para generar PWM por software en el MCP23017
  Controlar la velocidad del motor derecho según el joystick 
  */


  int potenciaBraso = joystickAPWM(ctl->axisRX());
    if (potenciaBraso > 30 ) { 
      movimientoBraso = potenciaBraso;//potenciaBraso;
      Serial.printf("Potencia de braso = %d\n", potenciaBraso);
      
        }else if(potenciaBraso < -30 ) {
    movimientoBraso = potenciaBraso;
  } else {  // Detener
      
     movimientoBraso = 0;
    }





 
  /*control de el Motor de Codo
  Función para generar PWM por software en el MCP23017
  Controlar la velocidad del motor derecho según el joystick 
  */

  int potenciaCodo = joystickAPWM(ctl->axisRY());
    if (potenciaCodo > 30) { 
      //movimientoCodo = potenciaCodo;
      movimientoCodo = potenciaCodo;
      Serial.printf("Potencia de codo = %d\n", potenciaCodo);
      
    }else if(potenciaCodo < -30) {
    movimientoCodo = potenciaCodo;
  }
      
     else {  // Detener
      
     movimientoCodo = 0;
    }


  /*control de el Motor de Pala
  Función para generar PWM por software en el MCP23017
  Controlar la velocidad del motor derecho según el joystick 
  */

  int potenciaPala = joystickAPWM(ctl->axisY());
    if (potenciaPala > 30) { 
     
      movimientopala = potenciaPala;
      Serial.printf("Potencia de la pala = %d\n", potenciaPala);
      
    }else if(potenciaPala < -30) {
    movimientopala = potenciaPala;
    }
      
    else {  // Detener
      movimientopala = 0;
      }





}

void setup() {

  Serial.begin(115200);
  // Inicializa el bus I2C con los pines 12 (SDA) y 13 (SCL)
  Wire.begin(13, 12);
  // pin de entrrada de sensores de inicio y fin carrera
  pinMode(iBrazo, INPUT);
  pinMode(fBrazo, INPUT);
  pinMode(iPala, INPUT);
  pinMode(fPala, INPUT);


 // Inicializa el MCP23017
  if (!mcp.begin_I2C()) {  // También puedes pasar la dirección I2C si no es la predeterminada
    Serial.println("No se encuentra el MCP23017. Revisa la conexión.");
    while (1);
  }
  for (int i = 0; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
  }

  //put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("Dirección BT: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

}



void loop() {


  //Motor derecho
  pwmMotor(rightMotor0,rightMotor1, aceleradorD);

  //Motor izquierdo
  pwmMotor(leftMotor0,leftMotor1, aceleradorI);

  //Motor de Giro

  pwmMotor(motorGiro1,motorGiro2, aceleradorG);

  //Motor de Braso


  pwmControl(motorBraso1,motorBraso2, movimientoBraso, iBrazo, fBrazo);
 


  //Motor de Codo

  pwmMotor(motorCodo1,motorCodo2, movimientoCodo);

  // Motor pala


  pwmControl(motorPala1, motorPala2, movimientopala, iPala, fPala);
  




  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    
    processControllers();
  }

  else { vTaskDelay(1); 
  }
 
 
} 
// Función para generar PWM por software en el MCP23017, para 2 pines de motores
  void pwmMotor(int pin1, int pin2, int potenciaEntrada) {
  



    if(potenciaEntrada == 0){
      //Motor apagado
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
    }else if(potenciaEntrada >= 1 ) {
      mcp.digitalWrite(pin1, HIGH);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      /*delayMicroseconds(potenciaEntrada * 50);// Ajuste para simular PWM

      // Apagar el Motor durante la otra parte del ciclo
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      
      delayMicroseconds((255 - potenciaEntrada) * 50);  // Ajuste para simular PWM*/
    }else if(potenciaEntrada <= -1 ) {
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, HIGH);// Motor -
      /*delayMicroseconds(potenciaEntrada * -50);  // Ajuste para simular PWM

      // Apagar el Motor durante la otra parte del ciclo
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      delayMicroseconds((255 + potenciaEntrada)* 50);  // Ajuste para simular PWM*/
    } 
  }
// Función para generar PWM por software en el MCP23017, para 2 pines de motores
  void pwmControl(int pin1, int pin2, int potenciaEntrada, int iPin, int fPin) {
  



    if(potenciaEntrada == 0){
      //Motor apagado
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      
    }else if(potenciaEntrada >= 1) {
      mcp.digitalWrite(pin1, HIGH);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
     /* delayMicroseconds(potenciaEntrada * 40);// Ajuste para simular PWM

      // Apagar el Motor durante la otra parte del ciclo
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      delayMicroseconds((255 - potenciaEntrada) * 40);  // Ajuste para simular PWM
      */
    }else if(potenciaEntrada <= -1) {
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, HIGH);// Motor -
     /* delayMicroseconds(potenciaEntrada * -40);  // Ajuste para simular PWM

      // Apagar el Motor durante la otra parte del ciclo
      mcp.digitalWrite(pin1, LOW);// Motor +
      mcp.digitalWrite(pin2, LOW);// Motor -
      delayMicroseconds((255 + potenciaEntrada)* 40);  // Ajuste para simular PWM*/
    } 
  }
