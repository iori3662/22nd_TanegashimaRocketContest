import serial
import serial.tools.list_ports

# 'COM2' 9600bps Parityなしの場合
Serial_Port=serial.Serial(port='COM2', baudrate=9600, parity= 'N')

#送信(tx)
data=input()+'\r\n'
data=data.encode('utf-8')
Serial_Port.write(data)

#受信(rx)
data=Serial_Port.readline() # 1byte受信なら data=Serial_Port.read(1)
data=data.strip()
data=data.decode('utf-8')
print(data)
