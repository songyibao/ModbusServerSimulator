# Modbus 服务端模拟
## 安装依赖
```
apt install libmodbus5 libmodbus-dev
```

## 使用方式
```register.csv```文件格式，以点位```40101```为例  
|寄存器类型|偏移量|值|
|-|-|-|
|HOLDING|100|123|

地址偏移量指相对于的偏移量，如点位 40101的地址偏移量为40101-40001=100  
仅支持```HOLDING```保持寄存器


