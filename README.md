# Implementación para Red Pitaya usando repo red-pitaya-notes

## Configuración

Usando la información de la sección *Getting Started* del siguiente link

https://pavel-demin.github.io/red-pitaya-notes/playground/https://pavel-demin.github.io/red-pitaya-notes/playground/

cargo la imagen de Linux y uso el kernel de microbamba en el visual studio code.

La única configuración en Ubuntu debe ser marcar en IPv4 de la conexión wired *Share to other computers* para poder acceder a las aplicaciones de Red Pitaya desde

http://rp-f09168.local/

## Pruebas

Se consiguió enviar un pulso desde una interfaz master a la interfaz slave a través de 2 FIFOs y a través de los conversores DA/AD, marcando cuando la información esta lista desde un bit de configuración usado como flag

Para probarlo:

  - En la carpeta de notebooks desde el visual studio code copiar *HUB_MASTER_pulse_FIFO.py*, *loopback_FIFO_3.bit*, *HUB_MASTER_pulse_ADC.py* y *loopback_ADC_3.bit*.

  - Entrar al link http://rp-f09168.local/ y marcar *playground* para programar la FPGA por primera vez, asegurarse que se encienda el LED azul.

  - Correr el script de python que se quiera probar desde visual studio code.

## Diseño en Vivado

### FIFO Loopback
![Alt text](Vivado_FIFO_loopback.png)

### ADC Loopback
![Alt text](Vivado_ADC_loopback.png)

# Conexión ethernet con PYNQ-Z2

## Configuración en Vitis

Para el wrapper asegurarse que en el BSP del archivo vitis-comp.json inlcuyan las librerías lwip213 y xiltimer.

![Alt text](Wrapper_config.png)

## Configuración de conexión Wired en Ubuntu

Configurar la dirección IP del cliente como cualquiera dentro de la red 192.168.1.0 manualmente y desactivar IPv6, el resto mantener automático.

![Alt text](IPv4.png)

![Alt text](IPv6.png)

## Lector de UART

Para ver los mensajes de progreso y lo recibido en la PYNQ uso la herramienta de minicom con:

```bash
sudo minicom -b 115200 -o -D /dev/ttyUSB1
```

## Pruebas

En terminal arrancar minicom con PYNQ enchufada y encendida y luego correr el código de lwip_udp_perf_server_ethernet desde vitis.

Una vez completada la conexión (avisa por UART) correr alguno de los scripts de Python. Asegurarse que la IP destino sea 192.168.1.0, que el puerto sea 5001 y completar la línea de filename con algún archivo de video en su PC.

  - Para UDP_Client_send.py:

      Envía "Hola mundo" hasta Ctrl+C del programa.

  - Para envío de video:

      -   sender_completo.py:

            Una vez corrido el programa, darle al botón de play, se muestra el video transmitido y desde la UART se leen los paquetes recibidos.
            PROBLEMA --> Error de alocación del buffer en el handler de la interrupción, no da a basto (ver xaxiemacif_dma.c de la librería lwip213, buscar "unable to alloc pbuf in recv_handler" en Vitis).

      -   sender_segmentado.py:
   
            Funciona igual que el anterior pero intenta segmentar el fotograma antes de enviarlo.
            PROBLEMA --> Se cuelga el video debido a la segmentación según el tamaño de paquete, editable en línea 39.

      
    
