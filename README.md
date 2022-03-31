# zongheshiyan
stm32综合实验设计24l01与nrf905
实验板：stm32f103zet6
To other students
## main是nrf905的版本
   此处摘引了wdomisk的HAL库代码https://github.com/wdomski/NRF905-STM32-example.git
   注意905不需要接uclk、am、dr，只接一个地，剩下10个引脚若有需求的请修改gpio和spi3，定时器采用了tim1.
## 24l01的版本
   同理也需要注意gpio和spi，我的板子有现成的接口了.
