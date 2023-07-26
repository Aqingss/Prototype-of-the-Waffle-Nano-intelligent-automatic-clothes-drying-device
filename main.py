import utime
import machine
from machine import I2C, Pin, SPI, PWM
from a010000 import A010000
#传感器引脚定义
i2c = I2C(1, sda=Pin(0), scl=Pin(1), freq=40000)
i2c_add = 0x5B
i2cAddress = i2c.scan()
print(i2cAddress)
#引用电机驱动
stp=A010000()
#蜂鸣器引脚定义
pwm = PWM(0, Pin(7))
pwm.init(freq=256,duty=0)

is_forward = True # 默认初始在室外

class Bme:
    def __init__(self):
        self.Hum = 0

bme = Bme()
Lux = 0
is_outdoor = True

def setup():
    utime.sleep(0.001)

def loop():
    global Lux, is_outdoor
    steps_per_rotation = 150
    while True:
        # 根据is_outdoor的值判断当前状态，True表示初始状态（室外），False表示下雨或天黑状态（室内）
        if is_outdoor: 
            # 判断环境条件，获取湿度和光强度
            get_bme()
            Lux = get_lux()
            # 在没有雨且白天的情况下，保持不变，继续检测环境
            while bme.Hum / 100 < 80 and Lux / 100 > 200:
                get_bme()
                print("HUM : {:.2f} %".format(bme.Hum / 100))
                Lux = get_lux()
                print("Lux: {:.2f} lux".format(Lux / 100))
                utime.sleep(0.2)
            # 检测到下雨或天黑，切换到is_outdoor为False状态，即室内状态
            if bme.Hum / 100 > 80 or Lux / 100 < 200:
                is_outdoor = not is_outdoor
                # 启动蜂鸣器
                pwm.duty(95)
                pwm.freq(1048)
                # 启动电机
                stp.set_step_time(2000)
                stp.enable()
                stp.rel_angle(60) # 顺时针转动60度
                # 关闭蜂鸣器
                pwm.duty(0)
        elif not is_outdoor:
            # 在室内状态下，同样检测湿度和光强度
            get_bme()
            Lux = get_lux()
            # 在下雨或天黑的情况下，保持不变，继续检测环境
            while bme.Hum / 100 > 80 or Lux / 100 < 200:
                get_bme()
                print("HUM : {:.2f} %".format(bme.Hum / 100))
                Lux = get_lux()
                print("Lux: {:.2f} lux".format(Lux / 100))
                utime.sleep(0.2)
            # 检测到雨停且为白天，切换到is_outdoor为True的状态，即室外状态
            if bme.Hum / 100 < 80 and Lux / 100 > 200:
                is_outdoor = not is_outdoor

                pwm.duty(95)
                pwm.freq(1048)

                stp.set_step_time(2000)
                stp.enable()
                stp.rel_angle(-60) #逆时针转动60度
                
                pwm.duty(0)
        utime.sleep(0.2)
# 获取湿度
def get_bme():
    data = bytearray(10)
    data_16 = bytearray(2)
    data = iic_read(0x04, data, 10)
    data_16 = [(data[2] << 8) | data[3], (data[4] << 8) | data[5]]
    bme.Hum = (data[6] << 8) | data[7]
#获取光强度
def get_lux():
    data = bytearray(10)
    data_16 = bytearray(2)
    data = iic_read(0x00, data, 4)
    data_16 = [(data[0] << 8) | data[1], (data[2] << 8) | data[3]]
    Lux = ((data_16[0]) << 16) | data_16[1]
    return Lux

def iic_read(reg, data, length):
    utime.sleep_us(10)
    if length > 4:
        data = i2c.readfrom_mem(i2c_add, reg, 10)
    else:
        data = i2c.readfrom_mem(i2c_add, reg, 4)
    return data

setup()
loop()
