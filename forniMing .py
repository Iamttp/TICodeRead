# 用于匿名的openmv，串口通信程序
import sensor, image, time, math, struct
from pyb import UART,LED,Timer

#初始化镜头
sensor.reset()                                     #初始化摄像头，reset()是sensor模块里面的函数
sensor.set_pixformat(sensor.GRAYSCALE)             #设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.QQVGA)                 #图像质量  分辨率为120*160
sensor.skip_frames(30)
sensor.set_auto_gain(True)                          #自动增益
sensor.set_auto_whitebal(True)                      #打开白平衡
sensor.set_contrast(3)                              #对比度

blue_led  = LED(3)
clock = time.clock()#初始化时钟
uart = UART(3,500000)#初始化串口 波特率 500000

thresholds = [0, 70]#自定义灰度阈值
fthresholds = [70, 256]#自定义灰度阈值

# up_roi   = [0,   0, 80, 15]#上采样区0
# down_roi = [0, 55, 80, 15]#下采样区0
# left_roi = [0,   0, 25, 60]#左采样区0
# righ_roi = [55, 0,  25, 40]#右采样区0

up_roi   = [0,   0, 160, 30]#上采样区0
down_roi = [0, 110, 160, 30]#下采样区0
left_roi = [0,   0, 50, 120]#左采样区0
righ_roi = [110, 0,  50, 80]#右采样区0

class Dot(object):
    x = 0
    y = 0
    pixels = 0
    num = 0
    ok = 0
    flag = 0

class singleline_check():
    ok = 0
    flag1 = 0
    flag2 = 0
    rho_err = 0
    theta_err = 0

dot  = Dot()

dot  = Dot()
up   = singleline_check()
down = singleline_check()
left = singleline_check()
righ = singleline_check()
line = singleline_check()
singleline_check = singleline_check()

#点检测数据打包
def pack_dot_data():

    dot_x = 80 - dot.x
    dot_y = dot.y - 60

    # dot_x,dot_y 的右移一般为0，最后两位不要用（用于校验和）
    # 0x06 为传输数据长度（算上一个0x00），0x00,0x00用于校验数据（必须）
    # 适用于0x41模式，按照匿名自己的解析依次传入的flag,flag(应该是sta),x,y
    pack_data=bytearray([0xaa,0x29,0x05,0x41,0x07,
        dot.flag,dot.flag,
        dot_x>>8,dot_x,
        dot_y>>8,dot_y,0x00,0x00])

    lens = len(pack_data)#数据包大小
    i = 0
    sum = 0

    #和校验
    while i<(lens-1):
        sum = sum + pack_data[i]
        i = i+1
    pack_data[lens-1] = sum % 256
    # 飞控代码的校验和大概为这样
    return pack_data

#线检测数据打包
def pack_linetrack_data():
    rho = int(singleline_check.rho_err)
    theta = int(singleline_check.theta_err)

    # 有些数据还未用到，但是为了统一。
    pack_data=bytearray([0xaa,0x29,0x05,0x42,0x07,
        line.flag,line.flag,
        rho>>8,rho,
        theta>>8,theta,
        0x00,0x00])

    #清零线检测偏移数据和倾角数据，使得在没有检测到线时，输出为零
    singleline_check.rho_err = 0
    singleline_check.theta_err = 0
    singleline_check.flag1 = 0

    lens = len(pack_data)#数据包大小
    i = 0
    sum = 0

    #和校验
    while i<(lens-1):
        sum = sum + pack_data[i]
        i = i+1
    pack_data[lens-1] = sum % 256

    return pack_data

def fine_border(img,area,area_roi):
    #roi是“感兴趣区”通过设置不同的感兴趣区，可以判断线段是一条还是两条，是T型线，还是十字、还是7字线
    singleline_check.flag1 = img.get_regression([(255,255)],roi=area_roi, robust = True)
    if (singleline_check.flag1):
        area.ok=1

#找线
def found_line(img):
    fine_border(img,up,up_roi) #上边界区域检测
    fine_border(img,down,down_roi) #下边界区域检测
    fine_border(img,left,left_roi) #左边界区域检测
    fine_border(img,righ,righ_roi) #右边界区域检测

    line.flag = 0
    if up.ok:
        line.flag = line.flag | 0x01 #将line.flag最低位置1
    if down.ok:
        line.flag = line.flag | 0x02 #将line.flag第2位置1
    if left.ok:
        line.flag = line.flag | 0x04 #将line.flag第3位置1
    if righ.ok:
        line.flag = line.flag | 0x08 #将line.flag第4位置1
    #print(line.flag)     #做测试用，在正常检测时最好屏蔽掉

    #对图像所有阈值像素进行线性回归计算。这一计算通过最小二乘法进行，通常速度较快，但不能处理任何异常值。 若 robust 为True，则将
    #使用泰尔指数。泰尔指数计算图像中所有阈值像素间的所有斜率的中值。thresholds：追踪的颜色范围
    singleline_check.flag2 = img.get_regression([(0,0)], robust = True)
    if (singleline_check.flag2):
        #print(clock.fps())
        singleline_check.rho_err = abs(singleline_check.flag2.rho())-0 #求解线段偏移量的绝对值
        # TODO 180-
        singleline_check.theta_err = singleline_check.flag2.theta()-0
        #在图像中画一条直线。singleline_check.flag2.line()意思是(x0, y0)到(x1, y1)的直线；颜色可以是灰度值(0-255)，或者是彩色值
        #(r, g, b)的tupple，默认是白色
        img.draw_line(singleline_check.flag2.line(), color = 127)
    #清零标志位
    up.ok = down.ok = left.ok = righ.ok = 0
    up.num = down.num = left.num = righ.num = 0
    up.pixels = down.pixels = left.pixels = righ.pixels = 0


    if line.flag != 0:
        res = pack_linetrack_data()
        print(res)
        uart.write(res)

#点检测函数
def check_dot(img):
    #thresholds为黑色物体颜色的阈值，是一个元组，需要用括号［ ］括起来可以根据不同的颜色阈值更改；pixels_threshold 像素个数阈值，
    #如果色块像素数量小于这个值，会被过滤掉area_threshold 面积阈值，如果色块被框起来的面积小于这个值，会被过滤掉；merge 合并，如果
    #设置为True，那么合并所有重叠的blob为一个；margin 边界，如果设置为5，那么两个blobs如果间距5一个像素点，也会被合并。
    for blob in img.find_blobs([thresholds], pixels_threshold=150, area_threshold=150, merge=True, margin=5):
        if dot.pixels<blob.pixels():#寻找最大的黑点
            #对图像边缘进行侵蚀，侵蚀函数erode(size, threshold=Auto)，size为kernal的大小，去除边缘相邻处多余的点。threshold用
            #来设置去除相邻点的个数，threshold数值越大，被侵蚀掉的边缘点越多，边缘旁边白色杂点少；数值越小，被侵蚀掉的边缘点越少，边缘
            #旁边的白色杂点越多。
            img.erode(2)
            dot.pixels=blob.pixels() #将像素值赋值给dot.pixels
            dot.x = blob.cx() #将识别到的物体的中心点x坐标赋值给dot.x
            dot.y = blob.cy() #将识别到的物体的中心点x坐标赋值给dot.x
            dot.ok= 1
            #在图像中画一个十字；x,y是坐标；size是两侧的尺寸；color可根据自己的喜好设置
            img.draw_cross(dot.x, dot.y, color=127, size = 10)
            #在图像中画一个圆；x,y是坐标；5是圆的半径；color可根据自己的喜好设置
            img.draw_circle(dot.x, dot.y, 5, color = 127)

    #判断标志位
    dot.flag = dot.ok

    #清零标志位
    dot.pixels = 0
    dot.ok = 0

    #发送数据
    res = pack_dot_data()
    #print(res)
    uart.write(res)

i = 0
while(True):
    i+=1
    clock.tick()
    img = sensor.snapshot()
    ##先对图像进行分割，二值化，将在阈值内的区域变为白色，阈值外区域变为黑色
    img.binary([fthresholds])

    #点检测
    check_dot(img)
    if i%10:
        found_line(img)
    blue_led.on()
    if i%50:
        blue_led.off()
