# RL78_F24_IICA0_I2C_Slave_interrupt
 RL78_F24_IICA0_I2C_Slave_interrupt

1. initial F24 EVB , to test IICA0 slave ( refer from RL78_F24_Boot_loader_UART_CRC)

- UART0 : P15/TX , P16/RX , to printf message

- IICA0 : P62/SCL , P63/SDA , to receive I2C master protocol (RA6 as I2C master)

2. press keyboard : 3 to entry slave receive flow

below is debug log when receive first block data and last block data from I2C master

![image](https://github.com/released/RL78_F24_IICA0_I2C_Slave_interrupt/blob/main/log1.jpg)

![image](https://github.com/released/RL78_F24_IICA0_I2C_Slave_interrupt/blob/main/log2.jpg)

