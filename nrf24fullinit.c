//-@ TODO implement receiving packets in IRQ pin's callback func

#include "main.h"
#include "nrf24fullinit.h"
//#include 'stm32f3xx_hal.h'
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;

// Инициализирует порты 
void radio_port_init() {
  //RADIO_DDR |= (1 << RADIO_CSN) | (1 << RADIO_CE); // Ножки CSN и CE на выход
  //RADIO_DDR &= ~(1 << RADIO_IRQ); // IRQ - на вход
  csn_deassert;
  radio_deassert_ce;
  //spi_init();
}

uint8_t spi_recvByte()
{
	uint8_t buf = 0;
	HAL_SPI_Receive(&hspi2,&buf,1,1000);
	return buf;
}

uint8_t spi_send_recv(uint8_t addr)
{
  uint8_t val=0;
  HAL_SPI_TransmitReceive(&hspi2,&addr,&val,1,1000);
  return val;
}

// Выполняет команду cmd, и читает count байт ответа, помещая их в буфер buf, возвращает регистр статуса
uint8_t radio_read_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
  csn_assert;
  uint8_t status = spi_send_recv(cmd);
  HAL_SPI_Receive(&hspi2,buf,count,1000);

//  while (count--) {
//  *(buf++) = spi_send_recv(0xFF);
//  }
  csn_deassert;
  return status;
}

// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
uint8_t radio_write_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
  csn_assert;
  uint8_t status = spi_send_recv(cmd);

  HAL_SPI_Transmit(&hspi2, buf, count, 1000);
  //while (count--) {
  //  spi_send_recv(*(buf++));
  //}
  csn_deassert;
  return status;
}

// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
uint8_t radio_readreg(uint8_t reg)
{
  reg = reg & 0x1F;
  uint8_t dt=0, dummy = 0b11111111;
  csn_assert;

  //STATUS
  HAL_SPI_TransmitReceive(&hspi2,&reg, &dt, 1,1000); //STATUS
  if (reg!=STATUS)
  {
    HAL_SPI_TransmitReceive(&hspi2, &dummy, &dt,1,1000); // STATUS
  }
  csn_deassert;
  return dt;
}

// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
uint8_t radio_writereg(uint8_t reg, uint8_t val) {
  csn_assert;
  //uint8_t status = spi_send_recv((reg & 31) | W_REGISTER);
  uint8_t status = 0;
  //spi_send_recv(val);
  reg = (reg & 31) | W_REGISTER;

  HAL_SPI_Transmit(&hspi2,&reg, 1,1000); // STATUS

  HAL_SPI_Transmit(&hspi2,&val,1,1000);
  csn_deassert;
  return status;
}

// Читает count байт многобайтового регистра reg (от 0 до 31) и сохраняет его в буфер buf,
// возвращает регистр статуса
uint8_t radio_readreg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
  return radio_read_buf((reg & 31) | R_REGISTER, buf, count);
}

// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
uint8_t radio_writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
  return radio_write_buf((reg & 31) | W_REGISTER, buf, count);
}

// Возвращает размер данных в начале FIFO очереди приёмника
uint8_t radio_read_rx_payload_width() {
  csn_assert;
  spi_send_recv(R_RX_PL_WID);
  uint8_t answ = spi_send_recv(0xFF);
  csn_deassert;
  return answ;
}

// Выполняет команду. Возвращает регистр статуса
uint8_t radio_cmd(uint8_t cmd) {
  csn_assert;
  uint8_t status = spi_send_recv(cmd);
  csn_deassert;
  return status;
}

// Возвращает 1, если на линии IRQ активный (низкий) уровень.
uint8_t radio_is_interrupt()
{
  //return (RADIO_PIN & RADIO_IRQ) ? 0 : 1;
	return (GPIO_PinState)IRQ_PinState;
}


// Функция производит первоначальную настройку устройства. Возвращает 1, в случае успеха, 0 в случае ошибки
uint8_t radio_init()
{
  uint8_t self_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Собственный адрес
  uint8_t remote_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}; // Адрес удалённой стороны
  uint8_t chan = 3; // Номер радио-канала (в диапазоне 0 - 125)

  radio_deassert_ce;
  HAL_Delay(5);
  for(uint8_t cnt = 100;;)
  {

    radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)); // Выключение питания
	  HAL_Delay(5);
	  if (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)))
      {break;}
    // Если прочитано не то что записано, то значит либо радио-чип ещё инициализируется, либо не работает.
    if (!cnt--)
      return 0; // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
    HAL_Delay(1);
  }

  radio_writereg(EN_AA, (1 << ENAA_P1)); // включение автоподтверждения только по каналу 1
  radio_writereg(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); // включение каналов 0 и 1
  radio_writereg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); // выбор длины адреса 5 байт
  radio_writereg(SETUP_RETR, SETUP_RETR_DELAY_250MKS | SETUP_RETR_UP_TO_2_RETRANSMIT); 
  radio_writereg(RF_CH, chan); // Выбор частотного канала
  radio_writereg(RF_SETUP, RF_SETUP_1MBPS | RF_SETUP_0DBM); // выбор скорости 1 Мбит/с и мощности 0dBm
  
  radio_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5); // Подтверждения приходят на канал 0 
  radio_writereg_buf(TX_ADDR, &remote_addr[0], 5);

  radio_writereg_buf(RX_ADDR_P1, &self_addr[0], 5);
  
  radio_writereg(RX_PW_P0, 0);
  radio_writereg(RX_PW_P1, 32); 
  radio_writereg(DYNPD, (1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
  radio_writereg(FEATURE, 0x04); // разрешение произвольной длины пакета данных

  radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX)); // Включение питания
  return (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX))) ? 1 : 0;
}

// Вызывается, когда превышено число попыток отправки, а подтверждение так и не было получено.
void on_send_error() {
 // TODO здесь можно описать обработчик неудачной отправки
}

// Вызывается при получении нового пакета по каналу 1 от удалённой стороны.
// buf - буфер с данными, size - длина данных (от 1 до 32)
//I've made it "weak" so you can redefine it in man.c
__weak void on_packet(uint8_t * buf, uint8_t size) {
 // TODO здесь нужно написать обработчик принятого пакета

 // Если предполагается немедленная отправка ответа, то необходимо обеспечить задержку ,
 // во время которой чип отправит подтверждение о приёме
 // чтобы с момента приёма пакета до перевода в режим PTX прошло:
 // 130мкс + ((длина_адреса + длина_CRC + длина_данных_подтверждения) * 8 + 17) / скорость_обмена
 // При типичных условиях и частоте МК 8 мГц достаточно дополнительной задержки 100мкс
}

// Помещает пакет в очередь отправки.
// buf - буфер с данными, size - длина данных (от 1 до 32)
uint8_t send_data(uint8_t * buf, uint8_t size) {
  radio_deassert_ce; // Если в режиме приёма, то выключаем его
  uint8_t conf = radio_readreg(CONFIG);
  if (!(conf & (1 << PWR_UP))) // Если питание по какой-то причине отключено, возвращаемся с ошибкой
    return 0;
  uint8_t status = radio_writereg(CONFIG, conf & ~(1 << PRIM_RX)); // Сбрасываем бит PRIM_RX
  if (status & (1 << TX_FULL_STATUS))  // Если очередь передатчика заполнена, возвращаемся с ошибкой
    return 0;
  radio_write_buf(W_TX_PAYLOAD, buf, size); // Запись данных на отправку
  radio_assert_ce; // Импульс на линии CE приведёт к началу передачи
  HAL_Delay(1); //_delay_us(15); // Нужно минимум 10мкс, возьмём с запасом
  radio_deassert_ce;
  return 1;
}

void check_radio() {
  if (!radio_is_interrupt()) // Если прерывания нет, то не задерживаемся
    return;
  uint8_t status = radio_cmd(NOP);
  radio_writereg(STATUS, status); // Просто запишем регистр обратно, тем самым сбросив биты прерываний

  if (status & ((1 << TX_DS) | (1 << MAX_RT))) { // Завершена передача успехом, или нет,
    if (status & (1 << MAX_RT)) { // Если достигнуто максимальное число попыток
      radio_cmd(FLUSH_TX); // Удалим последний пакет из очереди
      on_send_error(); // Вызовем обработчик
    }
    if (!(radio_readreg(FIFO_STATUS) & (1 << TX_EMPTY))) { // Если в очереди передатчика есть что передавать
      radio_assert_ce; // Импульс на линии CE приведёт к началу передачи
      HAL_Delay(1); // Нужно минимум 10мкс, возьмём с запасом
      radio_deassert_ce;
    } else {
      uint8_t conf = radio_readreg(CONFIG);
      radio_writereg(CONFIG, conf | (1 << PRIM_RX)); // Устанавливаем бит PRIM_RX: приём
      radio_assert_ce; // Высокий уровень на линии CE переводит радио-чип в режим приёма
    }
  }
  uint8_t protect = 4; // В очереди FIFO не должно быть более 3 пакетов. Если больше, значит что-то не так
  while (((status & (7 << RX_P_NO)) != (7 << RX_P_NO)) && protect--) { // Пока в очереди есть принятый пакет
    uint8_t l = radio_read_rx_payload_width(); // Узнаём длину пакета
    if (l > 32) { // Ошибка. Такой пакет нужно сбросить
      radio_cmd(FLUSH_RX);
    } else {
      uint8_t buf[32]; // буфер для принятого пакета
      radio_read_buf(R_RX_PAYLOAD, &buf[0], l); // начитывается пакет
      if ((status & (7 << RX_P_NO)) == (1 << RX_P_NO)) { // если datapipe 1
        on_packet(&buf[0], l); // вызываем обработчик полученного пакета
      }
    }
    status = radio_cmd(NOP);
  }
}
