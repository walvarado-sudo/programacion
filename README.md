<h1 style="color:#4B0082; text-align:center; background-color:#E6E6FA; padding:12px; border-radius:12px;">
✨ Taller de Programación y Descripción de Hardware ✨
</h1>

---

<h2 style="color:#9932CC;text-align:center;">1-. Comparativa entre Lenguaje de Programación y Lenguaje de Descripción de Hardware</h2>

<table style="width:100%; border-collapse:collapse; border:2px solid #BA55D3; border-radius:8px; overflow:hidden;">
  <tr style="background-color:#D8BFD8; text-align:center; color:#000;">
    <th>Característica</th>
    <th>Lenguaje de Programación (C, Python, etc.)</th>
    <th>Lenguaje de Descripción de Hardware (VHDL, Verilog, etc.)</th>
  </tr>
  <tr style="background-color:#E6E6FA; color:#000;">
    <td><b>Propósito</b></td>
    <td>Crear <b>software</b> que se ejecuta en un procesador.</td>
    <td>Diseñar <b>hardware digital</b> (circuitos lógicos, registros, compuertas).</td>
  </tr>
  <tr style="background-color:#FFFFFF; color:#000;">
    <td><b>Nivel de abstracción</b></td>
    <td>Alto (algoritmos, estructuras, lógica de control).</td>
    <td>Bajo o medio (comportamiento de señales eléctricas).</td>
  </tr>
  <tr style="background-color:#E6E6FA; color:#000;">
    <td><b>Ejecución</b></td>
    <td>Secuencial.</td>
    <td>Paralela.</td>
  </tr>
  <tr style="background-color:#FFFFFF; color:#000;">
    <td><b>Resultado</b></td>
    <td>Programa ejecutable.</td>
    <td>Diseño sintetizable en un chip FPGA o ASIC.</td>
  </tr>
  <tr style="background-color:#E6E6FA; color:#000;">
    <td><b>Ejemplo de uso</b></td>
    <td>Programar un microcontrolador.</td>
    <td>Diseñar un decodificador o controlador lógico.</td>
  </tr>
</table>

<div style="background-color:#F3E5F5; padding:12px; border-left:5px solid #BA55D3; margin-top:10px; border-radius:6px;">
<b>💡 Conclusión:</b><br>
Un lenguaje de programación controla hardware <b>existente</b>, mientras que un lenguaje de descripción de hardware <b>define el hardware</b> que se implementará.
</div>

---

<h2 style="color:#9932CC;text-align:center;">2-. Comparativa entre Microcontrolador, Microprocesador, FPGA y PLD</h2>

<table style="width:100%; border-collapse:collapse; border:2px solid #BA55D3; border-radius:8px; overflow:hidden;">
  <tr style="background-color:#D8BFD8; text-align:center; color:#000;">
    <th>Dispositivo</th>
    <th>Descripción</th>
    <th>Ventajas</th>
    <th>Desventajas</th>
    <th>Ejemplo</th>
  </tr>
  <tr style="background-color:#E6E6FA;">
    <td><b>Microcontrolador (MCU)</b></td>
    <td>Chip con CPU, memoria y periféricos integrados.</td>
    <td>Bajo costo, fácil de programar, ideal para sistemas embebidos.</td>
    <td>Menor capacidad de procesamiento.</td>
    <td>ESP32, Arduino Uno.</td>
  </tr>
  <tr style="background-color:#FFFFFF;">
    <td><b>Microprocesador (CPU)</b></td>
    <td>Solo la unidad de procesamiento central.</td>
    <td>Alta potencia y rendimiento.</td>
    <td>Requiere componentes externos (RAM, ROM, periféricos).</td>
    <td>Intel i5, ARM Cortex-A.</td>
  </tr>
  <tr style="background-color:#E6E6FA;">
    <td><b>FPGA</b> (Field Programmable Gate Array)</td>
    <td>Circuito digital reconfigurable mediante lógica programable.</td>
    <td>Alta velocidad, ejecución paralela, personalizable.</td>
    <td>Programación compleja, más caro.</td>
    <td>Xilinx Basys 3, Altera DE10.</td>
  </tr>
  <tr style="background-color:#FFFFFF;">
    <td><b>PLD</b> (Programmable Logic Device)</td>
    <td>Dispositivo lógico programable de menor escala.</td>
    <td>Rápido, flexible, bajo consumo.</td>
    <td>Menor capacidad que un FPGA.</td>
    <td>GAL, CPLD.</td>
  </tr>
</table>

---

<h2 style="color:#9932CC;text-align:center;">3-. Lenguajes más usados en Programación de Hardware</h2>

<div style="background-color:#E6E6FA; padding:10px; border-radius:10px; border-left:5px solid #BA55D3; margin-bottom:10px;">
  <b>💻 VHDL</b><br>
  <b>Tipo:</b> Lenguaje de Descripción de Hardware<br>
  <b>Uso:</b> Diseño estructural y lógico de circuitos digitales.
</div>

<div style="background-color:#F3E5F5; padding:10px; border-radius:10px; border-left:5px solid #9932CC; margin-bottom:10px;">
  <b>💻 Verilog</b><br>
  <b>Tipo:</b> Lenguaje de Descripción de Hardware<br>
  <b>Uso:</b> Diseño digital más sintético y rápido de escribir.
</div>

<div style="background-color:#E0BBE4; padding:10px; border-radius:10px; border-left:5px solid #8A2BE2; margin-bottom:10px;">
  <b>🧠 SystemVerilog</b><br>
  <b>Tipo:</b> HDL avanzado<br>
  <b>Uso:</b> Simulación y verificación de sistemas digitales complejos.
</div>

<div style="background-color:#F8F0FC; padding:10px; border-radius:10px; border-left:5px solid #DA70D6; margin-bottom:10px;">
  <b>🧩 C / C++ / MicroPython</b><br>
  <b>Tipo:</b> Lenguajes de Programación<br>
  <b>Uso:</b> Control de microcontroladores (MCU, ESP32, etc.).
</div>

<div style="background-color:#E6E6FA; padding:10px; border-radius:10px; border-left:5px solid #BA55D3;">
  <b>⚙️ MATLAB HDL Coder</b><br>
  <b>Tipo:</b> Generador automático<br>
  <b>Uso:</b> Conversión de modelos a código HDL sintetizable.
</div>

---

<h2 style="color:#9932CC;text-align:center;">4-.Propuesta de Solución Tecnológica Embebida (Ejemplo: ESP32)</h2>

<div style="background-color:#F8F0FC; padding:10px; border-radius:10px; border-left:5px solid #8A2BE2;">
<b>💡 Idea:</b> Diseñar un sistema de <b>monitoreo ambiental</b> con un <b>ESP32</b> que mida temperatura, humedad y calidad del aire, enviando los datos por Wi-Fi a un servidor local.
</div>

### 🔧 Componentes:
- ESP32 (microcontrolador)
- Sensor DHT11 (temperatura y humedad)
- Sensor MQ135 (calidad del aire)
- Servidor Web (interfaz de visualización)

### 💻 Código (Ejemplo en Arduino IDE)
```cpp
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.print(" °C  Humedad: ");
  Serial.print(h);
  Serial.println(" %");
  delay(2000);
}
```
<div style="background-color:#E6E6FA; padding:10px; border-radius:10px;">

### ✅ Ventajas:
- Bajo costo.
- Conectividad Wi-Fi y Bluetooth.
- Fácil programación (compatible con Arduino IDE).

### ❌ Desventajas:
- Limitado en procesamiento.
- No apto para operaciones paralelas avanzadas.
</div>

<h2 style="color:#9932CC; text-align:center;">5-. Programación de Hardware (Ejemplo: Basys 3 con FPGA)</h2> <div style="background-color:#F3E5F5; padding:10px; border-radius:10px;"> 🎯 <b>Objetivo:</b> Diseñar un <b>contador binario de 4 bits</b> en FPGA usando <b>VHDL</b>. </div>


### 💻 Código VHDL:
```vhdl
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity contador is
  Port ( clk : in STD_LOGIC;
         q : out STD_LOGIC_VECTOR (3 downto 0));
end contador;

architecture Behavioral of contador is
  signal count : STD_LOGIC_VECTOR (3 downto 0) := "0000";
begin
  process(clk)
  begin
    if rising_edge(clk) then
      count <= count + 1;
    end if;
  end process;
  q <= count;
end Behavioral;
```
<div style="background-color:#E6E6FA; padding:10px; border-radius:10px;">

### ✅ Ventajas:
- Alta velocidad de ejecución.
- Procesamiento **paralelo real**.
- Totalmente reconfigurable.

### ❌ Desventajas:
- Programación compleja (requiere conocimiento de HDL).
- Mayor costo y consumo energético.
</div>

<h2 style="color:#9932CC;text-align:center;">5.1-. Conclusión Solucion tecnologica</h2> <table style="width:100%; border-collapse:collapse; border:2px solid #BA55D3; border-radius:8px;"> 
<tr style="background-color:#D8BFD8; text-align:center; color:#000;"> <th>Sistema</th> <th>Tipo</th> <th>Ventajas</th> <th>Desventajas</th> </tr> <tr style="background-color:#E6E6FA;">
 <td><b>Microcontrolado (ESP32)</b></td> <td>Hardware + Software</td> <td>Económico, fácil de programar, ideal para IoT.</td> <td>Limitado en procesamiento.</td> </tr> <tr style="background-color:#FFFFFF;">
  <td><b>Programación en FPGA (Basys 3)</b></td> <td>Hardware reconfigurable</td> <td>Alta velocidad y paralelismo.</td> <td>Costoso y complejo de programar.</td> </tr> </table> 
  
  <div style="background-color:#F3E5F5; padding:12px; border-left:5px solid #8A2BE2; margin-top:10px; border-radius:6px;"> 🧾 <b>Conclusión final:</b><br> Los microcontroladores son ideales para sistemas embebidos simples ademas de conectados, mientras que las FPGA son más adecuadas para aplicaciones de procesamiento paralelo o hardware especializado. </div>



<h2 style="color:#4B0082;text-align:center;">📚 Referencias</h2>
- Xilinx Basys 3 Reference Manual – Digilent Inc.  
- Espressif Systems. (2024). *ESP32 Technical Reference Manual*.  
- Douglas Perry, *VHDL Programming by Example*, McGraw-Hill.