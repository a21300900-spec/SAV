# PWM Library for STM32F446RE

This example shows how to configure TIM3 to produce a 1.8 kHz PWM signal with
software limits for duty-cycle control between 7 % and 94 %. The code is
structured as a small reusable library (`tim3_pwm`) that can be called from
`main.c`.

## Files

- `inc/tim3_pwm.h`: Public API for the PWM helper.
- `src/tim3_pwm.c`: Implementation of the TIM3 PWM configuration and runtime
  helpers.
- `src/main.c`: Minimal example demonstrating how to use the helper from the
  application.

## Uso directo en proyectos simples

1. Asegúrate de que los controladores HAL de STM32 estén en tu proyecto y que
   las rutas de inclusión apunten al directorio `inc`.
2. Llama a `TIM3_PWM_Init()` con el canal deseado (el canal 1 corresponde al
   pin PA6 en la placa Nucleo-F446RE).
3. Usa `TIM3_PWM_SetDutyCycle()` para actualizar el ciclo útil; la librería
   aplica automáticamente los límites entre 7 % y 94 %.

Ajusta `SystemClock_Config()` para que coincida con el árbol de relojes de tu
proyecto si no usas la configuración generada por CubeMX.

## Integración en STM32CubeIDE

1. **Genera el proyecto base en CubeMX/CubeIDE.**
   - Selecciona la placa STM32F446RE o el microcontrolador equivalente.
   - Configura el reloj del sistema (por ejemplo, HSI a 16 MHz o HSE a 84 MHz)
     según tus necesidades.
   - No es necesario activar TIM3 ni sus GPIO desde CubeMX, porque la librería
     se encarga de habilitar los relojes y configurar los pines.
   - Genera el código y abre el proyecto en STM32CubeIDE.

2. **Copia los archivos de la librería.**
   - Añade `inc/tim3_pwm.h` al directorio `Core/Inc/` (o crea una carpeta
     `Drivers/Custom/inc` y agrega esa ruta a los includes).
   - Añade `src/tim3_pwm.c` al directorio `Core/Src/` (o al que prefieras para
     tu código de aplicación).
   - Si guardas los archivos en una carpeta personalizada, abre
     *Project Properties → C/C++ General → Paths and Symbols* y añade la ruta
     al apartado *Includes* de GCC.

3. **Incluye la cabecera en `main.c`.**
   ```c
   #include "tim3_pwm.h"
   ```

4. **Inicializa el temporizador tras `SystemClock_Config()`.**
   Inserta el siguiente bloque en la sección `/* USER CODE BEGIN 2 */` de
   `main.c`:
   ```c
   if (TIM3_PWM_Init(TIM_CHANNEL_1) != HAL_OK)
   {
       Error_Handler();
   }
   if (TIM3_PWM_SetDutyCycle(TIM_CHANNEL_1, 50.0f) != HAL_OK)
   {
       Error_Handler();
   }
   ```
   Sustituye `TIM_CHANNEL_1` por el canal que quieras usar:
   - Canal 1 → PA6
   - Canal 2 → PA7
   - Canal 3 → PB0
   - Canal 4 → PB1

5. **Compila y programa.**
   - La librería calcula automáticamente el período para 1.8 kHz en función de
     la frecuencia real del bus APB1, por lo que no necesitas ajustar
     manualmente el valor del ARR.
   - Ajusta el ciclo útil en tiempo de ejecución mediante
     `TIM3_PWM_SetDutyCycle()` dentro del bucle principal o en interrupciones.

6. **Opcional:** Si tu aplicación utiliza la función débil `HAL_TIM_MspPostInit`
   generada por CubeMX para configurar los pines de PWM, puedes dejarla vacía o
   eliminar la configuración redundante. La librería inicializa los GPIO antes
   de arrancar el PWM, por lo que no habrá conflictos.

Con estos pasos tendrás el PWM de TIM3 funcionando en tu proyecto de
STM32CubeIDE con los límites de ciclo útil definidos.
