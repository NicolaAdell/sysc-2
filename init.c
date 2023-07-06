#include "init.h"

// Variable para guardar el valor de temperatura
float temperatura = 0.0;
// Variable para guardar el valor del setpoint
float setpoint = 0.0;

/*
 * @brief Inicializacion de perifericos
 */

 void init(void) {
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  gpio_put(1);
  sleep(3000);
  // Inicializacion de UART
  stdio_init_all();
  // Creo un repeating timer
  struct repeating_timer timer;
  // Creo un callback para la interrupcion del timer
  add_repeating_timer_ms(ADC_DELAY_MS, muestreo_periodico, NULL, &timer);
  // Configuro el I2C0 a 100 KHz de clock
  i2c_init(i2c0, 100 * 1000);
  // Elijo GPIO4 como linea de SDA
  gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
  // Elijo GPIO5 como linea de SCL
  gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
  // Activo pull-up en ambos GPIO, son debiles por lo que
  // es recomendable usar pull-ups externas
  gpio_pull_up(SDA_GPIO);
  gpio_pull_up(SCL_GPIO);
  // Inicializo LCD
  lcd_init();
  // Limpio LCD
  lcd_clear();
  // Escribo lo basico
  lcd_string("TEMP=");
  lcd_set_cursor(1, 0);
  lcd_string("SETP=");
  // Inicializo ADC
  adc_init();
  // Inicializo el GPIO del NTC como entrada analogica
  adc_gpio_init(NTC_GPIO);
  // Inicializo el GPIO del potenciometro como entrada analogica
  adc_gpio_init(SP_GPIO);
  // Inicializacion de PWM
  gpio_set_function(PWM_ENFRIAMIENTO, GPIO_FUNC_PWM);
  gpio_set_function(PWM_CALEFACCION, GPIO_FUNC_PWM);
  uint8_t slice = pwm_gpio_to_slice_num(PWM_ENFRIAMIENTO);
  pwm_config config = pwm_get_default_config();

  // - 100 Hz de frecuencia
  pwm_config_set_clkdiv (&config, 125);
  pwm_config_set_wrap(&config, 10000);
  pwm_init(slice, &config, true);

  // - Ancho de pulso inicial del 0%
  pwm_set_gpio_level(PWM_ENFRIAMIENTO, 0);
  pwm_set_gpio_level(PWM_CALEFACCION, 0);
}

/*
 * @brief Callback para la interrupcion de timer
 * @param t: puntero a repeating_timer
 */
bool muestreo_periodico(struct repeating_timer *t) {
  // Canal elegido, no se pierde el valor aunque sea local (static)
  static uint8_t selected_channel = 0;
  // Constante de proporcionalidad de NTC
  const uint16_t beta = 3950;
  // Cambio de canal 0 a 1 o viceversa
  adc_select_input(selected_channel ^ 1);
  // Lectura analogica (variable adc_value)
  uint16_t adc_value = adc_read();
  // Verifico que canal lei
  if(selected_channel == 0) {
    // Canal 0: termistor. Calcular valor de temperatura (variable temperatura)
    temperatura = 1 / (log(1 / (4095. / adc_value - 1)) / beta + 1.0 / 298.15) - 273.15;
  }
  else {
    // Canal 1: potenciometro. Calcular el valor de temperatura equivalente (4095 = 35 grados)
    setpoint = adc_value * 35 / 4095.;
    // Calcular error (temperatura - setpoint)
    float error = temperatura - setpoint;
    // Llamar al control proporcional
    float control_p = control_proporcional(error);
    // Verificar el signo del control
    if(control_p < 0) {
      // Si es negativo: temperatura > setpoint -> enfrio
      // Apagar un canal de PWM
      pwm_set_gpio_level(PWM_CALEFACCION, 0);
      // Encender el otro
      pwm_set_gpio_level(PWM_ENFRIAMIENTO, control_p);
      
    }
    else {
      // Si es positivo: temperatura < setpoint -> caliento
      // Apagar un canal de PWM
      pwm_set_gpio_level(PWM_ENFRIAMIENTO, 0);
      // Encender el otro
      pwm_set_gpio_level(PWM_CALEFACCION, control_p);
    }
  }
}
