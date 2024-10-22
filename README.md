## TITA_HW

> 利用Ubuntu socket_can[https://docs.kernel.org/networking/can.html]驱动和基于STM32 G0的candleLight FD[https://linux-automation.com/en/products/candlelight-fd.html]方案的usb2canfd实现上位机控制TITA机器人。

#### 实现功能

- [x] 读取TITA上传的电机，IMU，遥控器数据
- [ ] 读取TITA上传的电池信息

- [ ] 利用sdk控制TITA站立和运动
- [ ] 利用sdk单独控制电机
- [ ] 利用sdk控制TITA灯光系统



#### 硬件要求

- Ubuntu22.04电脑 （==20.04上socketcan驱动暂不支持canfd协议！==）
- usb2canfd



#### 代码编译

- 拉取代码

```
git clone git@github.com:L-SY/TITA_SDK.git
```

- 编译（采用cmake）

```
cd TITA_SDK/tita_hw/
mkdir build && cd build
cmake ..
make
```



#### 使用方法

- 正确的电气连接（当usb2canfd通过usb-a连接到电脑后可以被时别为`can`设备,可通过`ip link show`来查看是否存在，

应该类似如下显示:

```bash
can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN group default qlen 10
    link/can
```

- 初始化can

> 需要下载`can-utils` ： `sudo apt-get install can-utils`

```bash
sudo ip link set can0 up type can bitrate 100000 dbitrate 5000000 fd on
```

其中bitrate后的1M为canfd的仲裁域速度，dbitrate的5M为数据域速度速度。需要根据具体情况设置（示例和TITA规定一致）

如果前两步正确，新开终端利用`can-utils`进行测试:`candump can0`，步骤无问题应该看到滚动的can消息帧

- 运行测试代码

```bash
./test_peripheral
```
