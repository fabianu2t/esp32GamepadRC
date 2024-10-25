Descripción General
Este código está diseñado para controlar una pala mecánica a través de un control de videojuegos (Xbox o PlayStation). La pala mecánica cuenta con 6 motores, 4 sensores de fin de carrera y utiliza un ESP32 para la comunicación y control, un expansor de pines MCP23X17, y controladores de motores DRV8833 para manejar los motores.

Hardware Utilizado
ESP32: Microcontrolador principal que se encarga de la conexión con el control de juegos y la gestión de los motores.
MCP23X17: Expansor de pines I2C que permite más salidas digitales para controlar los motores.
DRV8833: Controlador de motores para manejar los 6 motores de la pala mecánica.
4 Sensores de Fin de Carrera: Detectan las posiciones extremas de los motores para evitar sobrecargas o daños.
Control de Xbox o PlayStation: Permite el control remoto de la pala.
Componentes del Código
Librerías Importadas

#include <ESP32Servo.h>  // Maneja servomotores con ESP32
#include <Bluepad32.h>    // Maneja la conexión con controles de juegos
#include "Adafruit_MCP23X17.h"  // Controla el MCP23X17
#include <Wire.h>         // Comunicación I2C


Definición de Pines

// Motores de giro
#define motorGiro1 14
#define motorGiro2 15
// Motores del brazo 
#define motorBraso1 10
#define motorBraso2 11
#define motorCodo1 3
#define motorCodo2 2
// Motor de la pala
#define motorPala1 9
#define motorPala2 8
// Motores de movimiento
#define leftMotor0 7
#define leftMotor1 6
#define rightMotor0 4
#define rightMotor1 5
// Pines de sensores de fin de carrera
#define iBrazo 35
#define fBrazo 34
#define iPala 33
#define fPala 25



Aquí se definen los pines utilizados para controlar los motores y los sensores de fin de carrera. Los motores están conectados al MCP23X17, mientras que los sensores están conectados directamente al ESP32.

Funciones para Manejo del Control
joystickAPWM y aceleradorAPWM: Estas funciones convierten los valores de los joysticks y del acelerador en señales PWM (modulación por ancho de pulso) para controlar la velocidad de los motores.

onConnectedController y onDisconnectedController: Manejan la conexión y desconexión de los controles de juego, permitiendo que el ESP32 reconozca cuando un control se conecta o se desconecta.

processControllers y processGamepad: Procesan la información de los controles conectados, como la posición de los joysticks y los botones presionados, para controlar los motores de la pala.

Control de Motores

void pwmMotor(int pin1, int pin2, int potenciaEntrada) {
    if (potenciaEntrada == 0) {
        mcp.digitalWrite(pin1, LOW);
        mcp.digitalWrite(pin2, LOW);
    } else if (potenciaEntrada > 0) {
        mcp.digitalWrite(pin1, HIGH);
        mcp.digitalWrite(pin2, LOW);
    } else {
        mcp.digitalWrite(pin1, LOW);
        mcp.digitalWrite(pin2, HIGH);
    }
}
