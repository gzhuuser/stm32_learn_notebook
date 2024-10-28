# STM32

![image-20240928094843788](.\img\image-20240928094843788.png)

STM32命名规制

![image-20240928104404671](img\image-20240928104404671.png)





## Contex-M介绍

ARM公司: 只做内核设计和IP授权,不参与芯片设计

### ARM架构

![image-20240928093136977](img\image-20240928093136977.png)





### 数据手册查看

手册下载:

- ST中文社区网：https://www.stmcu.org.cn/
- ST官网：https://www.st.com

STM32F103开发板的参数

![image-20240928111851746](img\image-20240928111851746.png)

在我们开发的过程中,重点关注两个模块

1.  引脚模块
2. 电气特征

![image-20240928112445129](img\image-20240928112445129.png)

这两个单元能方便我们去了解这个开发板的特征,以及引脚的分布图



电源引脚分布:

![image-20240928112904213](img\image-20240928112904213.png)

引脚类型:

1. 电源引脚, 带V的基本都是
2. 晶振引脚:23-24, 晶振引脚
3. 复位引脚: 25, NRST
4. 下载引脚: 三类下载引脚
   - JTAG: 占用5个IO口
   - SWD:占用2个IO口, 可以仿真调试
   - 串口下载:占用两个IO口, 只能下载程序不能调试
   - ![image-20240929161434419](img\image-20240929161434419.png)
5. BOOT引脚
   - 138:BooT0
   - 48: BooT1
6. GPIO引脚

![image-20240929162116316](img\image-20240929162116316.png)





## STM32的最小系统

![image-20240929163046514](img\image-20240929163046514.png)

![image-20240929164109498](img\image-20240929164109498.png)

### 电源部分

![image-20240929164148405](img\image-20240929164148405.png)



<img src="C:\Users\31168\AppData\Roaming\Typora\typora-user-images\image-20240929164219485.png" alt="image-20240929164219485" style="zoom:50%;" />

> [!NOTE]
>
> 这里使用了一个电压稳压器,为了将电源控制在3.3V左右,防止5V电压直接击穿电路

这里经过稳压后,给多组电源供电,方便提供给不同设备供电



![image-20240929165128190](img\image-20240929165128190.png)

当我们开发板断电的时候,由下面部分的纽扣电池来供电,以维持一些数据的存储 ,额定电压为3V

![image-20240929165216295](img\image-20240929165216295.png)



### 复位电路

![image-20240929165351519](img\image-20240929165351519.png)

STM32复位引脚NRST保持低电平状态时间1~4.5ms即可复位

- 没按下RESET之前,是3.3v直接供电,处于高电平状态, 按下按键后,并联分流,这时NRST就是处于低电平状态,就会复位了
- 也可以直接通电复位,当我们直接供电的时候,C12电容(10**4 PF)还未满,需要充电,这时候是导通的,也起到分流作用,此时NRST也是低电平, 充满时间是满足1-4.5ms的





### BOOT启动电路

M3和M4内核

![image-20240929165819153](img\image-20240929165819153.png)

使用了一个短路帽来进行控制,如果按下就是接在13, 24, 这时候通的是高电压。35,46则是接地,也就是关机



M7内核

![image-20240929165838860](img\image-20240929165838860.png)



### 晶振电路

![image-20240929172521268](img\image-20240929172521268.png)



### 下载电路

![image-20240929172746330](img\image-20240929172746330.png)

![image-20240929172754752](img\image-20240929172754752.png)



### IO分配原则

优先特点设备IO,如SPI, 下载串口等,然后分配通用IO口,最后微调

通用IO: GPIO





## 环境配置

![image-20241003170535695](img\image-20241003170535695.png)

### 安装MDK

![image-20241003165759394](img\image-20241003165759394.png)

第三步下载算法对于F103的板子不需要

注意点:

1. 安装目录及路径不要有任何中文汉字，且路径越短越好
2. 电脑系统名和用户名最好都不要有任何中文



### 安装仿真器

直接下载CH340串口即可,用来使用串口下载

![image-20241003165948282](img\image-20241003165948282.png)

CH340C是在开发板上面的,所以我们要在PC下载CH340串口协议,这样才能正确下载

![image-20241003170314161](img\image-20241003170314161.png)



### MDK5编译例程

![image-20241003171136095](img\image-20241003171136095.png)

1. 全局编译: 编译全部文件
2. 部分编译: 只编译部分修改过的文件



![image-20241004155442814](img\image-20241004155442814.png)



### 串口下载程序

1. 串口下载程序须知
2. 串口下载程序的硬件连接
3. 配置下载工具(ATK-XISP.exe 正点原子自研)
4. STM32启动模式(M3和M4)



#### 下载须知

1. M3、M4、M7开发板支持串口下载程序， 但是ATK-XISP.exe软件只支持下载到内部FLASH
2. STM32的ISP下载，常用串口1下载程序
3. 因为使用USB虚拟串口，所以事先得安装CH340 USB虚拟串口驱动（搭建开发环境视频）





#### 下载程序的硬件连接

![image-20241004160409963](img\image-20241004160409963.png)

- 串口1之所以要进行短路帽连接,是因为PA9在stm32相当于TX, PA10相当于RX
- B0,B1通过短路帽连接到GND,低电平
- 电源关闭



#### 串口调试软件

![image-20241004161811633](img\image-20241004161811633.png)

1. 串口选择CH340
2. 选择Hex文件
3. 选择编译选项
4. 选择模式
5. 开始编程

在编程前可以先获取芯片信息看看是否有效,获取不了就换个波特率之类的

> [!NOTE]
>
> 每次重新下载或者获取芯片信息时都需要按下复位键:RESET





#### STM32的启动模式

![image-20241004162720732](img\image-20241004162720732.png)

ISP下载步骤:

1. BOOT0接高电平，BOOT1接低电平
2. 按复位键



执行步骤

1. BOOT0接低电平，BOOT1接任意
2. 按复位键



实际上,我们下载时是吧B0和B1都接0, 然后直接用一键下载电路进行下载的

![image-20241004163312326](img\image-20241004163312326.png)

一键下载电路，是利用串口的DTR和RTS信号，分别控制STM32的复位和BOOT0引脚，配合ATK-XISP.exe软件，设置：DTR低电平复位，RTS高电平进BootLoader，这样， BOOT0和STM32的复位引脚，完全由下载软件自动控制，从而实现一键下载。



不使用一键下载的电路:

![image-20241004163502621](img\image-20241004163502621.png)

可以看出RTS和DTR是没有接任何电路的





### DAP下载电路

![image-20241010161713719](img\image-20241010161713719.png)

DAP仿真下载器:

![image-20241010161915779](img\image-20241010161915779.png)





#### KEIL5配置DAP

我这里用的是F103,使用的ST-Link, 不是DAP,具体配置如下:

![image-20241010165302557](img\image-20241010165302557.png)

**![image-20241010165318776](img\image-20241010165318776.png)**

![image-20241010165332174](img\image-20241010165332174.png)





### JTAG/SWD调试模块

Cortex-M内核含有硬件调试模块，该模块可在取指(指令断点)或访问数据(数据断点)时停止。内核停止时，可以查询内核的内部状态和系统的外部状态。完成查询后，可恢复程序执行。

![image-20241010171124720](img\image-20241010171124720.png)



#### 断点调试

![image-20241010172421361](img\image-20241010172421361.png)



#### 查看程序段执行的时间

1. 设置开发板内核的时钟



![image-20241010172750793](img\image-20241010172750793.png)

F1开发板的时钟频率是72MHZ

这样就可以在调试的过程中看程序执行的时间了

![image-20241010173032893](img\image-20241010173032893.png)



#### 仿真结束可能出现的报错

![image-20241010173233609](img\image-20241010173233609.png)



#### Debug过程中工具栏常用功能

![image-20241010173352411](img\image-20241010173352411.png)



##### command

![image-20241010173604240](img\image-20241010173604240.png)

##### 反汇编窗口

![image-20241010173625559](img\image-20241010173625559.png)

##### 符号窗口

 ![image-20241010173701101](img\image-20241010173701101.png)



##### Registers寄存器窗口

![image-20241010173810622](img\image-20241010173810622.png)

##### Call Stack窗口

查看函数调用关系和局部变量

![image-20241010173956945](img\image-20241010173956945.png)

##### watch窗口

查看函数首地址或者变量的值

通过点击变量:add_watch来查看变量在调试过程中的值

**注意**: 有时候修改后,需要编译后再调式

![image-20241010181611426](img\image-20241010181611426.png)



##### memory窗口

可以通过这个窗口,看到数组内存的情况

![image-20241010181732654](img\image-20241010181732654.png)

M3/M4/M7内核是小端模式，内存的值得倒着读



##### Peripheral窗口：查看寄存器的值

![image-20241010182007222](img\image-20241010182007222.png)

调试时，使用该功能可以确定配置寄存器是否有问题







#### 仿真时注意的点

1. 仿真时，使用MDK的Level 0等级优化
2. 调试停止在断点处时，只是内核停止，外设会继续运行
3. 断点的设置要有时间观念，考虑是否会打断正常通信



![image-20241010182416260](img\image-20241010182416260.png)







## F1的系统架构

![image-20241011152134022](img\image-20241011152134022.png)

F1架构可以理解为下面的几个单元:

![image-20241011152233688](img\image-20241011152233688.png)

![image-20241011152447569](img\image-20241011152447569.png)

对于我们学习来说,最重要的是AHB系统总线, APB1和2总线,这几个总线负责了全部的外设

总线时钟频率：

1. AHB：72MHz (Max)
2. APB1：36MHz (Max)
3. APB2：72MHz (Max)





##  存储器映射

![image-20241013215154128](img/image-20241013215154128.png)



### 存储器映射

![image-20241013215432746](img/image-20241013215432746.png)

19根地址线,有$2^{19}$个地址,即512k

16根数据线,有2个字节的数据存储位置

所以整块芯片加起来有$2*512k=1024k=1Mb$的数据存储

映射方式:

<img src="img/image-20241013220010511.png" alt="image-20241013220010511" style="zoom:33%;" />



### 存储器功能划分:

STM32将存储器分成了8个块,他们各自的作用是:

| **存储块**  | **功能**          | **地址范围**                       |
| ----------- | ----------------- | ---------------------------------- |
| **Block 0** | Code（FLASH）     | 0x0000 0000 ~ 0x1FFF FFFF（512MB） |
| **Block 1** | SRAM              | 0x2000 0000 ~ 0x3FFF FFFF（512MB） |
| **Block 2** | 片上外设          | 0x4000 0000 ~ 0x5FFF FFFF（512MB） |
| **Block 3** | FSMC Bank1&2      | 0x6000 0000 ~ 0x7FFF FFFF（512MB） |
| **Block 4** | FSMC Bank3&4      | 0x8000 0000 ~ 0x9FFF FFFF（512MB） |
| **Block 5** | FSMC寄存器        | 0xA000 0000 ~ 0xBFFF FFFF（512MB） |
| **Block 6** | 没用到            | 0xC000 0000 ~ 0xDFFF FFFF（512MB） |
| **Block 7** | Cortex M3内部外设 | 0xE000 0000 ~ 0xFFFF FFFF（512MB） |

我们重点看Block0-2



#### Block0

![image-20241013220511076](img/image-20241013220511076.png)





#### Block1

![image-20241013220523960](img/image-20241013220523960.png)



#### Block2

![image-20241013220535940](img/image-20241013220535940.png)





## 寄存器映射

寄存器是单片机内部一种特殊的内存，可以实现对单片机各个功能的控制

简单来说：寄存器就是单片机内部的控制机构



可以理解为寄存器就是某个开关,用来控制设备进行工作

![image-20241013223150400](img/image-20241013223150400.png)



### STM32寄存器分类

![image-20241013223212891](img/image-20241013223212891.png)

### 寄存器映射

![image-20241013223435228](img/image-20241013223435228.png)

![image-20241013223631903](img/image-20241013223631903.png)

GPIO的命名通常是以A,B,C这样命名, 和51单片机不一样,51单片机以1,2,3这样命名



### 寄存器描述解读

![image-20241013223854886](img/image-20241013223854886.png)



### 寄存器计算

就一个公式:

- 总线基地址（BUS_BASE_ADDR）
- 外设基于总线基地址的偏移量（PERIPH_OFFSET）
- 寄存器相对外设基地址的偏移量（REG_OFFSET）

寄存器地址 = BUS_BASE_ADDR +  PERIPH_OFFSET + REG_OFFSET



计算例子

![image-20241014204651892](img/image-20241014204651892.png)

也可以使用结构体完成快速的地址定位

![image-20241014204744156](img/image-20241014204744156.png)







## 创建MDK工程

### 新建寄存器版本的MDK工程

1.   新建工程文件夹
2.   新建工程框架
3.   添加文件
4.   魔术棒设置
5.   添加main.c,并编写代码

![image-20241015201036080](img/image-20241015201036080.png)



#### 新建工程文件

![image-20241015201341892](img/image-20241015201341892.png)



#### 拷贝或者新建工程相关文件

![image-20241015201425537](img/image-20241015201425537.png)



![image-20241015201459049](img/image-20241015201459049.png)

该文件夹用于存放正点原子和其他第三方提供的中间层代码（组件/Lib 等），如： USMART、MALLOC、 TEXT、 FATFS、 USB、 LWIP、各种 OS、各种 GUI 等

![image-20241015201508481](img/image-20241015201508481.png)





### 全流程

#### 构建工程目录

![image-20241015213321641](img/image-20241015213321641.png)

##### 配置Drivers

![image-20241015213410395](img/image-20241015213410395.png)

CMSIS里面要找到一个启动函数,没有可以去找个例子直接复制



##### Middlewares

暂时用不到,为空



##### Output

用来存储编译后的文件



##### Project

存放用户程序,里面新建一个MDK-ARM文件夹



##### User 

存放用户的main函数





#### UE4的工程文件创建

![image-20241015213801627](img/image-20241015213801627.png)

选择MDK-ARM目录

![image-20241015213822147](img/image-20241015213822147.png)

然后选芯片

![image-20241015213847452](img/image-20241015213847452.png)

![image-20241015213854628](img/image-20241015213854628.png)

成功后:

![image-20241015213916731](img/image-20241015213916731.png)

##### 设置分组



![image-20241015214526561](img/image-20241015214526561.png)

![image-20241015214620690](img/image-20241015214620690.png)

设置完后的效果:

![image-20241015214628881](img/image-20241015214628881.png)



##### 设置魔术棒

![image-20241015213938284](img/image-20241015213938284.png)

##### target

![image-20241015214007078](img/image-20241015214007078.png)

MHz改8, 这个MDK版本有点高,所以不能改

ARM Compiler改成版本5



##### output

![image-20241015214146476](img/image-20241015214146476.png)

路径改成Output, 勾选Create Hex File





##### Listen

![image-20241015214212281](img/image-20241015214212281.png)

这个和output一样



![image-20241015214231189](img/image-20241015214231189.png)

1.   定义宏定义STM32F103xE
2.   优先级选0
3.   标准选C99
4.   Include Path选择编译时需要用到的头文件的路径

![image-20241015214336546](img/image-20241015214336546.png)



##### Debug

![image-20241015214357655](img/image-20241015214357655.png)

根据实际要求改

![image-20241015214406723](img/image-20241015214406723.png)

![image-20241015214428706](img/image-20241015214428706.png)\



##### utilties

![image-20241015214451740](img/image-20241015214451740.png)





这样就完成了,可以进行编译测试了







## HAL库的使用





### HAL库介绍



CMSIS: Cortex Microcontroller Software Interface Standard 微控制器软件接口标准, 由ARM和与其合作的公司指定的一套标准

![image-20241014205617446](img/image-20241014205617446.png)



为了方便开发,STM提供了3种库

- 标准外设库 (Standard Peripheral Libraries)
- HAL库(硬件抽象层)：Hardware Abstraction Layer 
- LL库：Low Layer



![image-20241014211145051](img/image-20241014211145051.png)

#### STM32Cube固件包

![image-20241014211417405](img/image-20241014211417405.png)

重点: 

1. 驱动源码
2. 中间文件
3. ST开发例程



![image-20241014213447476](img/image-20241014213447476.png)

![image-20241014213454441](img/image-20241014213454441.png)



CMSIS文件

![image-20241014214150672](img/image-20241014214150672.png)

![image-20241014214200677](img/image-20241014214200677.png)





### HAL库的框架结构

#### 比较重要的文件

![image-20241016211723970](img/image-20241016211723970.png)

#### API命名规则:

![image-20241016211815465](img/image-20241016211815465.png)

#### 对寄存器的操作API定义

![image-20241016212451417](img/image-20241016212451417.png)

#### 回调函数

![image-20241016212525668](img/image-20241016212525668.png)



### HAL库的构建

和MDK工程一样,只是在Device加一个HAL库的包







## STM32启动过程



### MAP文件

MAP文件是MDK编译代码后，产生的集程序、数据及IO空间的一种映射列表文件。简单说就是包括了：各种.c文件、函数、符号等的地址、大小、引用关系等信息。**分析各.c文件占用FLASH 和 RAM的大小，方便优化代码**

![image-20241016214322031](img/image-20241016214322031.png)



整个MAP反应了整个编译过程中的各种程序之间的引用关系,和变量之间的定义

交叉引用:

![image-20241016215840369](img/image-20241016215840369.png)



删除;

![image-20241016215817450](img/image-20241016215817450.png)

符号表:

![image-20241016215855594](img/image-20241016215855594.png)

内存映射

![image-20241016215911215](img/image-20241016215911215.png)

占用flash和SRAM的情况

![image-20241016215928808](img/image-20241016215928808.png)





### STM32启动模式

![image-20241017212737974](img/image-20241017212737974.png)

#### F1芯片启动模式

![image-20241017212930928](img/image-20241017212930928.png)

这里主要想表达的是启动地址相对于栈顶地址的相对地址为4个字节





### STM32启动过程

![image-20241017213807078](img/image-20241017213807078.png)

这里以FLASH启动为例, 将基地址映射到0x08000 0000 ,相对的PC计数器位置就是0X08000 0004 在这里获取到Reset_Handler函数的地址, 执行里面的启动文件, 这样一来系统就会自动调用我们的main函数

![image-20241017213941353](img/image-20241017213941353.png)





![image-20241018141046898](img/image-20241018141046898.png)







#### Reset_Handler函数介绍

![image-20241017214648937](img/image-20241017214648937.png)

汇编语言: 看定义来理解



![image-20241017214907617](img/image-20241017214907617.png)

堆栈设置可以去到启动文件设置







## STM32CuBeMX

stm32CuBeMx可以通过图形化界面来帮我们生成初始化代码

思路: 工具+不同芯片的STM32CuBe固件包来适配不同的型号的芯片



工具的使用:

### 关联固件包

先设定固件包的仓库

![image-20241018153343485](img/image-20241018153343485.png)

![image-20241018153355653](img/image-20241018153355653.png)



然后选择本地导入固件包或者下载

![image-20241018153420084](img/image-20241018153420084.png)

![image-20241018153440792](img/image-20241018153440792.png)





### 构建项目

![image-20241018153521687](img/image-20241018153521687.png)





#### 建立工程

![image-20241018153603875](img/image-20241018153603875.png)

选择芯片

![image-20241018153640988](img/image-20241018153640988.png)

双击即可

![image-20241018153651301](img/image-20241018153651301.png)

#### 配置时钟模块

![image-20241018153743329](img/image-20241018153743329.png)

在这里构建时钟模块





#### GPIO引脚配置

![image-20241018153848792](img/image-20241018153848792.png)

在这里找到我们要用的GPIO引脚,可以去查看对应的原理图

![image-20241018155717711](img/image-20241018155717711.png)

找到对应的引脚后可以来这里设置初始化的值



#### 设置Cortex内核

![image-20241018155644967](img/image-20241018155644967.png)

在这里完成配置

在设置Application时, Advanced和basic的区别是:

![image-20241018155930835](img/image-20241018155930835.png)

![image-20241018160017315](img/image-20241018160017315.png)



#### 生成项目代码

![image-20241018160045701](img/image-20241018160045701.png)





打开文件后的结构如下:

![image-20241018160103656](img/image-20241018160103656.png)

我们只需要编写main函数即可, 注意里面的注释,要在特定的位置写对应的内容,不然重新生成的时候会被覆盖



这样就完成建立了









## STM32的时钟系统



### 时钟系统的介绍

STM32时钟系统可以归纳为三个字: 选, 乘, 除



F1芯片中, 有四个产生时钟的震荡器

| 时钟源名称          | 频率      | 材料      | 用途       |
| ------------------- | --------- | --------- | ---------- |
| 高速外部振荡器(HSE) | 4~16MHz   | 晶体/陶瓷 | SYSCLK/RTC |
| 低速外部振荡器(LSE) | 32.768KHz | 晶体/陶瓷 | RTC        |
| 高速内部振荡器(HSI) | 8MHz      | RC        | SYSCLK     |
| 低速内部振荡器(LSI) | 40KHz     | RC        | RTC/IWDG   |



![image-20241018171206265](img/image-20241018171206265.png)

重点了解红色标记的部分



F1时钟树简图

![image-20241018171250416](img/image-20241018171250416.png)

名词的定义:

PLL(Phase-Locked-Loop): 锁相环

RCC: Reset and Clock Control

Osc: Oscillator*/*ˈɒsɪleɪtə(r)*/*振荡器

Periph: Peripheral*/*pəˈrɪfərəl*/*外围的





CubeMX上面的设置

![image-20241018171815370](img/image-20241018171815370.png)





### 系统时钟配置步骤

![image-20241018183708671](img/image-20241018183708671.png)

![image-20241018191254792](img/image-20241018191254792.png)



第2点很少需要调整

第5点只在H7芯片才有



#### 配置HSE_VALUE

![image-20241018190331328](img/image-20241018190331328.png)

F1默认8MHz



#### 配置HAL_RCC_OscConfig

![image-20241018191303258](img/image-20241018191303258.png)

这一部分对应着

![image-20241018191331243](img/image-20241018191331243.png)

```c
void sys_stm32_clock_init(uint32_t plln)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    RCC_OscInitTypeDef rcc_osc_init = {0};
    RCC_ClkInitTypeDef rcc_clk_init = {0};

    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;       /* Ñ¡ÔñÒªÅäÖÃHSE */
    rcc_osc_init.HSEState = RCC_HSE_ON;                         /* ´ò¿ªHSE */
    rcc_osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;          /* HSEÔ¤·ÖÆµÏµÊý */
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                     /* ´ò¿ªPLL */
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;             /* PLLÊ±ÖÓÔ´Ñ¡ÔñHSE */
    rcc_osc_init.PLL.PLLMUL = plln;                             /* PLL±¶ÆµÏµÊý */
    ret = HAL_RCC_OscConfig(&rcc_osc_init);                     /* ³õÊ¼»¯ */

    if (ret != HAL_OK)
    {
        while (1);                                              /* Ê±ÖÓ³õÊ¼»¯Ê§°Ü£¬Ö®ºóµÄ³ÌÐò½«¿ÉÄÜÎÞ·¨Õý³£Ö´ÐÐ£¬¿ÉÒÔÔÚÕâÀï¼ÓÈë×Ô¼ºµÄ´¦Àí */
    }

    /* Ñ¡ÖÐPLL×÷ÎªÏµÍ³Ê±ÖÓÔ´²¢ÇÒÅäÖÃHCLK,PCLK1ºÍPCLK2*/
    rcc_clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;        /* ÉèÖÃÏµÍ³Ê±ÖÓÀ´×ÔPLL */
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;               /* AHB·ÖÆµÏµÊýÎª1 */
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;                /* APB1·ÖÆµÏµÊýÎª2 */
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;                /* APB2·ÖÆµÏµÊýÎª1 */
    ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_2);  /* Í¬Ê±ÉèÖÃFLASHÑÓÊ±ÖÜÆÚÎª2WS£¬Ò²¾ÍÊÇ3¸öCPUÖÜÆÚ¡£ */

    if (ret != HAL_OK)
    {
        while (1);                                              /* Ê±ÖÓ³õÊ¼»¯Ê§°Ü£¬Ö®ºóµÄ³ÌÐò½«¿ÉÄÜÎÞ·¨Õý³£Ö´ÐÐ£¬¿ÉÒÔÔÚÕâÀï¼ÓÈë×Ô¼ºµÄ´¦Àí */
    }
}

```

1.   振荡器有4个,2个外部2个内部
2.   给对应的晶振器使能
3.   选中对应的分屏
4.   其他值暂时用不上,就不设置,默认为0
5.   配置PLL, PLL是锁相环, 需要配置里面的使能, PLL的输入(晶振来源), 增加频率的倍数
6.   最后赋值给HAL_RCC_OscConfig函数



**这些配置都有特定的寄存器来存放对应的值**



#### 配置HAL_RCC_ClockConfig函数

![image-20241018192352105](img/image-20241018192352105.png)

对应部分:

![image-20241018192438730](img/image-20241018192438730.png)

```c
void sys_stm32_clock_init(uint32_t plln)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    RCC_OscInitTypeDef rcc_osc_init = {0};
    RCC_ClkInitTypeDef rcc_clk_init = {0};

    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;       /* Ñ¡ÔñÒªÅäÖÃHSE */
    rcc_osc_init.HSEState = RCC_HSE_ON;                         /* ´ò¿ªHSE */
    rcc_osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;          /* HSEÔ¤·ÖÆµÏµÊý */
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                     /* ´ò¿ªPLL */
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;             /* PLLÊ±ÖÓÔ´Ñ¡ÔñHSE */
    rcc_osc_init.PLL.PLLMUL = plln;                             /* PLL±¶ÆµÏµÊý */
    ret = HAL_RCC_OscConfig(&rcc_osc_init);                     /* ³õÊ¼»¯ */

    if (ret != HAL_OK)
    {
        while (1);                                              /* Ê±ÖÓ³õÊ¼»¯Ê§°Ü£¬Ö®ºóµÄ³ÌÐò½«¿ÉÄÜÎÞ·¨Õý³£Ö´ÐÐ£¬¿ÉÒÔÔÚÕâÀï¼ÓÈë×Ô¼ºµÄ´¦Àí */
    }

    /* Ñ¡ÖÐPLL×÷ÎªÏµÍ³Ê±ÖÓÔ´²¢ÇÒÅäÖÃHCLK,PCLK1ºÍPCLK2*/
    rcc_clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;        /* ÉèÖÃÏµÍ³Ê±ÖÓÀ´×ÔPLL */
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;               /* AHB·ÖÆµÏµÊýÎª1 */
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;                /* APB1·ÖÆµÏµÊýÎª2 */
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;                /* APB2·ÖÆµÏµÊýÎª1 */
    ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_2);  /* Í¬Ê±ÉèÖÃFLASHÑÓÊ±ÖÜÆÚÎª2WS£¬Ò²¾ÍÊÇ3¸öCPUÖÜÆÚ¡£ */

    if (ret != HAL_OK)
    {
        while (1);                                              /* Ê±ÖÓ³õÊ¼»¯Ê§°Ü£¬Ö®ºóµÄ³ÌÐò½«¿ÉÄÜÎÞ·¨Õý³£Ö´ÐÐ£¬¿ÉÒÔÔÚÕâÀï¼ÓÈë×Ô¼ºµÄ´¦Àí */
    }
}

```

1.  配置系统时钟源:系统, HCLK(外设总线), PCLK1(APB1), PCLK2(APB2)。这里直接构成了一条频率路线
2.  配置系统时钟源: 这里的时钟源来自于PLL分频后
3.  设置分频系数



第二个参数FLatency表示的是FLASH等待的周期,FLASH的频率只有24MHz, 但是系统频率是72MHz, 所以需要等待2个是周期







## SYSTEM文件夹



### sys

![image-20241018194253633](img/image-20241018194253633.png)

具体功能作用和配置学到相关功能函数就知道了







### Deley

文件函数结构:

![image-20241018194533028](img/image-20241018194533028.png)

stm32主要是裸机,所以我们这里只学习不适用OS的版本



了解函数之前,先学习一下SysTick的工作原理

#### SysTick系统滴答定时器

![image-20241018194832628](img/image-20241018194832628.png)

LOAD可以由我们人为设定,这样就可以去控制时间,从而起到计时或者延时之类的效果



![image-20241018195537115](img/image-20241018195537115.png)

1.   COUNTFLAG: 溢出位判断, 如果溢出表示数到了0, 计数+1, 自动复位
2.   由于STM32已经固定了时钟源,所以这里就变成了分频
3.   TICKINT:中断请求时会用到
4.   ENABLE: 使能位



![image-20241018195910033](img/image-20241018195910033.png)

1.  RELOAD: 重载位的值, 有24位, 最大16xxxxx几
2.  CURRENT: 系统计数计数器,同样也是24位的



#### dealy函数

![image-20241018200555840](img/image-20241018200555840.png)

1.  CTRL设置为0是为了防止一开始初始化系统时,里面设置了系统计时器,从而产生不必要的影响因素
2.  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8):设置系统总线来源,这里说的是来源为HCLK(即APH总线)经过/8分频得到的
3.  g_fac_us表示的是sysclk/8后每1us占用多少频率





##### 微秒定时函数

这里的g_fac_us 的值为9, 因为在9MHZ的情况下, 每一个时钟等于$\frac{1}{9000000}$, 所以之后要乘以一个g_fac_us 变成$\frac{1}{1000000}$

```c
void delay_us(uint32_t nus) 
{ 
	uint32_t temp; 
	SysTick->LOAD = nus * g_fac_us; 	/* 时间加载 */ 
	SysTick->VAL = 0x00; 			/* 清空计数器 */ 
	SysTick->CTRL |= 1 << 0 ; 		/* 开始倒数 */ 
	do 
	{ 
		temp = SysTick->CTRL; 
	} while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLE位必须为1, 并等待时间到达 */

 
	SysTick->CTRL &= ~(1 << 0) ; 		/* 关闭SYSTICK */ 
	SysTick->VAL = 0X00; 			/* 清空计数器 */ 
}

```



##### 毫秒定时函数

```c
void delay_ms(uint16_t nms) 
{ 
	uint32_t repeat = nms / 1000;	/* 这里用1000,是考虑到可能有超频应用, 
							    	 * 比如128Mhz的时候, delay_us最大只能延时1048576us
								 */ 
	uint32_t remain = nms % 1000; 
	while (repeat) 
	{ 
		delay_us(1000 * 1000); 	/* 利用delay_us 实现 1000ms 延时 */ 
		repeat--; 
	} 
	if (remain) 
	{ 
		delay_us(remain * 1000); 	/* 利用delay_us, 把尾数延时(remain ms)给做了 */ 
	} 
}

```



这里ms使用的是us的定时函数,但是us的定时函数在超频的情况下, 最大可以延时到1048576, 约为1s, 所以如果delay_ms输入的值超过1s的话,就需要分开几次进行调用,就有了上面的代码



### usart











## GPIO

### GPIO的简介

GPIO: 通用输入输出口, 用来采集外部信息和控制外部器件, 以组为单位,每一组16个口,命名为: GPIOx_XXX()

![image-20241021201616867](img/image-20241021201616867.png)

这些就是一些GPIO某个组的部分引脚的口



特点:

1. 不同芯片信号IO口数量可能不一样
2. 快速翻转, 每次翻转最快只要两个周期,F1最快可到50Mhz (超频的情况下)
3. 每个IO口都可以用来做中断
4. 支持8种工作模式



电气特性:

stm32工作电压范围: 2-3.6V, 所以不能接5V给开发板,不然会烧毁开发板

GPIO识别电压范围: 

1. CMOS端口: -0.3V-1.164V和1.833V-3.6V
2. TTL(标有FT的都是TTL端口):5V, 3.3V



GPIO单个输出IO口最大电流是25mA



不同芯片的IO情况

![image-20241020222827422](img/image-20241020222827422.png)



### GPIO端口基本结构介绍

![image-20241020223313765](img/image-20241020223313765.png)

F1和其他系列的差距

#### F1结构模式

![image-20241020224708323](img/image-20241020224708323.png)

![image-20241020224810372](img/image-20241020224810372.png)



#### 斯密特触发器:

![image-20241020224830395](img/image-20241020224830395.png)

#### P-MOS, C-MOS

![image-20241020225101152](img/image-20241020225101152.png)

Mos需要有压差才能导通,这里VDD为1, 那么G就要为0,形成压差, 实际上1为3.3V,0为0V





### GPIO八种工作模式

我们可以将这八种工作模式分成四个输入,四个输出

| **GPIO**八种模式   | **特点及应用**                              |
| ------------------ | ------------------------------------------- |
| **输入浮空**       | 输入用，完全浮空，状态不定                  |
| **输入上拉**       | 输入用，用内部上拉，默认是高电平            |
| **输入下拉**       | 输入用，用内部下拉，默认是低电平            |
| **模拟功能**       | ADC、DAC                                    |
| **开漏输出**       | 软件IIC的SDA、SCL等                         |
| **推挽输出**       | 驱动能力强，25mA（max），通用输出           |
| **开漏式复用功能** | 片上外设功能（硬件IIC 的SDA、SCL引脚等）    |
| **推挽式复用功能** | 片上外设功能（SPI 的SCK、MISO、MOSI引脚等） |



#### 输入浮空

![image-20241021190148685](img/image-20241021190148685.png)

直接由外部环境决定,这里的IO是 



#### 输入上拉

![image-20241021191031202](img/image-20241021191031202.png)





#### 输入下拉

![image-20241021192121456](img/image-20241021192121456.png)



#### 模拟功能

![image-20241021192935685](img/image-20241021192935685.png)





#### 开漏输出

![image-20241021193406977](img/image-20241021193406977.png)

开漏时的输出控制为:

![image-20241021194559001](img/image-20241021194559001.png)



注意点:

1.   斯密特触发器打开: 表示着开漏输出时,可以输入
2.   P-MOS管始终关闭
3.   N-MoS为0表示I/O引脚为低电平, 这时候就能输出了
4.   外部如果想要输入高电平,这时候需要有一个外部上拉电阻才能输出高电平



#### 开漏服用功能

![image-20241021194219124](img/image-20241021194219124.png)





#### 推挽输出

![image-20241021194241526](img/image-20241021194241526.png)

输出控制里面的结构:

![image-20241021194435844](img/image-20241021194435844.png)

输出控制器里面有一个反相器, 输入1就是0, 输入0就是1



**驱动能力强**: 当P-MOS导通后, 直接由$V_{DD}$来给外部I/O引脚输出, 没有经过任何的电阻保护。这个时候就有了很强的驱动能力





#### 推挽式服用功能

![image-20241021194907318](img/image-20241021194907318.png)







F1和其他芯片的GPIO的差异点:

![image-20241021195001192](img/image-20241021195001192.png)





STM32输出内部是无法输入5V的电平的, 只有外界一个5V的上拉电阻才有机会输出5V





### GPIO寄存器

![image-20241021195812970](img/image-20241021195812970.png)

F1中,每个组有7个寄存器, 分别是GPIOx_CRL(x=A..E)

![image-20241021200020579](img/image-20241021200020579.png)



F1的话BRR和LCKR用的不多, 所以只需要关注CRL, CRH, IDR, ODR, BSRR这几个寄存器的作用

其他系列中的要关注7个寄存器,就是除了LCKR之外





#### CRL和CRH



![image-20241021200406132](img/image-20241021200406132.png)

![image-20241021200643070](img/image-20241021200643070.png)

两个寄存器,64个位,配置16个IO口,平均一个IO口由4个位决定



每个PIN口有CNF和MODE两个模式可以设置, 低位为MODE,高位为CNF, 根据上面的表来设置是输出还是输出, 是开漏还是推挽



![image-20241021203735939](img/image-20241021203735939.png)





#### ODR寄存器

![image-20241021203854621](img/image-20241021203854621.png)

以PA10为例, 这里对它的ODR10设置就可以控制上下拉输入

![image-20241021204125947](img/image-20241021204125947.png)

也就是这两个开关









#### IDR寄存器

![image-20241021204146726](img/image-20241021204146726.png)





#### BSRR设置/清除寄存器

![image-20241021204448036](img/image-20241021204448036.png)



对于F1芯片, 这个BSRR有32位,这32位用来控制ODR16位的寄存器

1.  31:16置1的话,对应的ODR16就为0, 15:0置1的话,对应的ODR16就为1,通过这种方式写入即可
2.  BSRR只对写入为1有效,如果写入0是没有任何影响的





#### 注意点

![image-20241021205904694](img/image-20241021205904694.png)





### 通用外设驱动模型(四步法)

![image-20241022115433306](img/image-20241022115433306.png)

时钟设置, 参数设置, **IO设置(USART), 中断设置**(这两个是可选的)



### GPIO配置步骤

1.   使能时钟
2.   设置工作模式
3.   设置输出状态
4.   读取输入状态

![image-20241022115816802](img/image-20241022115816802.png)





![image-20241022120157966](img/image-20241022120157966.png)

#### GPIO初始化时钟使能

![image-20241022120934025](img/image-20241022120934025.png)

最核心的是里面的SET_BIT这一段: 这是一个宏定义, 表示将APB2的某个GPIO口给使能了, 通过操作位, 第二个参数宏定义为 1<< x



#### GPIO_Init()

查看这些参数的范围的技巧: 去到是stm32f1xx_hal_gpio.h里面, 找到这个结构体,选择后面注释部分,ctrl+f全文件查找就能找到对于的可选值

![image-20241022185713759](img/image-20241022185713759.png)

Pin的可选值(F1):

![image-20241022185943619](img/image-20241022185943619.png)





Mode设置

![image-20241022190126588](img/image-20241022190126588.png)



Pull设置

上下拉设置

![image-20241022190719829](img/image-20241022190719829.png)

1.  浮空
2.  上拉
3.  下拉





Speed速度

![image-20241022190801920](img/image-20241022190801920.png)



#### writenPin

写入BSRR

```c
/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
  * @param  GPIO_Pin: specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState: specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16u;
  }
}
```



参数:

1.   GPIO的组
2.   对应组的端口
3.   设置高低电平



设置类型是枚举:

```c
typedef enum
{
  GPIO_PIN_RESET = 0u,
  GPIO_PIN_SET
} GPIO_PinState;
```



例子:

>   如果要设置PB10的口,那么参数为:
>
>   HAL_GPIO_WritenPin(GPIOB,GPIO_PIN_10, 1);







### 实战

#### 点亮LED灯

![image-20241022192236895](img/image-20241022192236895.png)

HAL库中,需要将所有外设的硬件设备放到Drivers/BSP中,然后设置分组

![image-20241022204400981](img/image-20241022204400981.png)

然后编写led.c和led.h

```c
// led.c

#include "./BSP/LED/led.h"


void led_init()
{
	// PB5 is LED IO
	GPIO_InitTypeDef gpio_init_struct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	gpio_init_struct.Pin = GPIO_PIN_5;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOB, &gpio_init_struct);
	// close the LED at init
	// In push-pull mode, when PinState is set to 1, the actual output is 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	
}

```

1.   使能时钟
2.   配置IO口属性
3.   初始化IO口电平值



led.h

```c
#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"
void led_init(void);

#endif

```

>   [!IMPORTANT]
>
>   一定要添加#include "./SYSTEM/sys/sys.h", 这个是初始化系统里面的库的,不导入会报一堆找不到库的错误



回到main函数,导入后写入即可

```c
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"


int main(void)
{
    HAL_Init();                                 /* ³õÊ¼»¯HAL¿â */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* ÉèÖÃÊ±ÖÓ,72M */
    delay_init(72);                             /* ³õÊ¼»¯ÑÓÊ±º¯Êý */
    led_init();                                 /* ³õÊ¼»¯LED */
    
    while(1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);                              
        delay_ms(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);                               
        delay_ms(500);
    }
}


```



也可以用翻转函数

```c
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"


int main(void)
{
    HAL_Init();                                 /* ³õÊ¼»¯HAL¿â */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* ÉèÖÃÊ±ÖÓ,72M */
    delay_init(72);                             /* ³õÊ¼»¯ÑÓÊ±º¯Êý */
    led_init();                                 /* ³õÊ¼»¯LED */
    
    while(1)
    {
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        delay_ms(200);
    }
}
```





#### 通过Key来控制一个LED灯



目的: 学习GPIO的输入功能

![image-20241022205126397](img/image-20241022205126397.png)

![image-20241022205135184](img/image-20241022205135184.png)

main函数

```c
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"


int main(void)
{
    HAL_Init();                                 /* ³õÊ¼»¯HAL¿â */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* ÉèÖÃÊ±ÖÓ,72M */
    delay_init(72);                             /* ³õÊ¼»¯ÑÓÊ±º¯Êý */
    led_init();                                 /* ³õÊ¼»¯LED */
	key_init();
    
    while(1)
    {
			if(key_scan())
			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
			}
			else
			{
				delay_ms(10);
			}
    }
}
```

记得初始化



key.c文件

```c
#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"

void key_init()
{
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOE_CLK_ENABLE();  
    
    gpio_init_struct.Pin = GPIO_PIN_4; 
    gpio_init_struct.Mode = GPIO_MODE_INPUT;  
    gpio_init_struct.Pull = GPIO_PULLUP;  
    
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);
}

// 

uint8_t key_scan(void)
{
	// key_down: ReadPin is 0
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0)
	{
        	// 防抖动
			delay_ms(10);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0)
			{
				while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0);
				return 1;
			}
				
	}
	return 0; 
}
```











## 中断

![image-20241023193644798](img/image-20241023193644798.png) 



中断使用的场景:

1.   实时控制: 在确定时间内做出对事件的响应
2.   故障处理: 检测到故障,第一时间处理
3.   数据传输: 收到外部的数据

>   [!CAUTION]
>
>   中断不会一直占用CPU资源





### GPIO外部中断简图

![image-20241023194251725](img/image-20241023194251725.png)









### NVIC

NVIC: Nested vectored interrupt controller 嵌套向量中断控制器,属于内核(M3/4/7)

![image-20241023194645797](img/image-20241023194645797.png)



随着型号越来越高,用到的外设资源越来越多,所有外部中断也就越来越多,每当发送中断后,CPU都会自动找到中断向量表中的服务函数来处理这些中断



#### 中断向量表:

![image-20241023194744539](img/image-20241023194744539.png)

可以在start文件中找到

![image-20241023194957841](img/image-20241023194957841.png)

#### NVIC相关寄存器

![image-20241023195116666](img/image-20241023195116666.png)

-   对于F1, ISER和ICER有16个位是保留的,因为属于内部,由CPU管理,这个只用来使能外部中断
-   IPR只用高四位,所有只有$2^4$个优先级,也就是16个中断优先级





#### 工作原理

![image-20241023195413207](img/image-20241023195413207.png)



#### 中断优先级

1.  抢占优先级(pre)：高抢占优先级可以打断正在执行的低抢占优先级中断
2.  响应优先级(sub)：当抢占优先级相同时，响应优先级高的先执行，但是不能互相打断(已经在执行的程序哪怕响应优先级高也不能被同级打断)
3.  抢占和响应都相同的情况下，自然优先级越高的，先执行
4.  自然优先级：中断向量表的优先级
5.  **数值越小，表示优先级越高**





#### 中断优先级分组

![image-20241023202825236](img/image-20241023202825236.png)

一个工程中一般只设置一次中断优先级分组



![image-20241023203355548](img/image-20241023203355548.png)

这句话的意思是: EXTI1和RTC可以打断EXTI0和Systick的执行,优先执行。而EXTI0和Systick则是以优先执行为原则,一旦有一方在执行,另一方就不能抢占, 没有执行的话,优先响应Systick



#### NVIC的使用

![](img/image-20241023203545141.png)





##### 设置分组:

```c
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  /* Check the parameters */
  assert_param(IS_NVIC_PRIORITY_GROUP(PriorityGroup));
  
  /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
  NVIC_SetPriorityGrouping(PriorityGroup);
}
```

![image-20241023204513274](img/image-20241023204513274.png)

##### 设置中断优先级:

```c
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
  uint32_t prioritygroup = 0x00U;
  
  /* Check the parameters */
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  
  prioritygroup = NVIC_GetPriorityGrouping();
  
  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}
```

![image-20241023205755640](img/image-20241023205755640.png)

输入参数:

-   IRQn_Type IRQn: 中断表的编号
-   uint32_t PreemptPriority 抢占优先级
-   uint32_t SubPriority 响应优先级



编号位置:

![image-20241023205328919](img/image-20241023205328919.png)





##### 设置使能中断

```c
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
  /* Check the parameters */
  assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

  /* Enable interrupt */
  NVIC_EnableIRQ(IRQn);
}
```

![image-20241023205941350](img/image-20241023205941350.png)

要给谁使能,那就将对应的IRQ编号发过去就行了







### EXTI

External(Extended) interrupt/event Controller，外部(扩展)中断事件控制器

它的主要作用是管理我们内部和外部的中断和事件处理, 如果是中断就进入NVIC, 如果是事件就自己处理了

![image-20241023210233458](img/image-20241023210233458.png)

![image-20241023210538299](img/image-20241023210538299.png)



 

![image-20241023212135831](img/image-20241023212135831.png)







#### EXTI的工作原理

![image-20241024212053005](img/image-20241024212053005.png)

1.   边缘检测电路由寄存器EXTI_RTSR和EXTI_FTSR中某个位置置1即可决定是上升沿触发,下降沿触发还是上升下降沿触发
2.   我们看到软件中断寄存器部分,这里是一个或门, 可以直接由软件中断来控制输出
3.   第二步中,出来的如果是1,那么会自动给EXTI_PR寄存器某个位置1, 然后给第三部分的与门输出1, 如果这时候中断屏蔽寄存器也输出1了,那就表示这个是一个中断事件, 需要到NVIC来处理
4.   事件屏蔽寄存器: 用来确定这个信号是事件, 让后面找到对应的外设去执行事件,一般很少用



##### RTSR上升沿触发器

![image-20241024213312552](img/image-20241024213312552.png)

F1只有20条输入线,所以只开放了20个端口



##### 下降沿触发

![image-20241024213345218](img/image-20241024213345218.png)

同理于上升沿寄存器





##### 软件中断事件寄存器

![image-20241024213412310](img/image-20241024213412310.png)





##### 挂起中断事件寄存器

![image-20241024213431526](img/image-20241024213431526.png)









#### AFIO

![image-20241025141904219](img/image-20241025141904219.png)

重点在外部中断配置: 由AFIO_EXTICR 1~4这四个寄存器来配置的





#### SYSCFG(F4/F7/H7)

外部中断配置: SYSCFG_EXTICR1-4个寄存器,配置和AFIO一样

![image-20241025142426800](img/image-20241025142426800.png)



#### EXTI和IO的映射关系

![image-20241024214024662](img/image-20241024214024662.png)

![image-20241025144355770](img/image-20241025144355770.png)

>   AFIO_EXTICR1~4表示4个32位寄存器,每个寄存器控制4条EXTI线,每条EXTI线由4个位控制(用于选择GPIO的端口)
>
>   1.   EXTI1-4控制16个GPIO的输出口
>   2.   每个IO口由4个位控制
>   3.   这四个位控制着是由哪个组IO口作为输出
>   4.   同一个EXTI线中,一个时间步只能通过一个组的对应的IO口





### 中断的使用

![image-20241025145252981](img/image-20241025145252981.png)

1.   外部中断:因为要通过GPIO口,而且来自外部,所以GPIO要设置为输入模式
2.   设置EXTI和IO的映射关系,告诉EXTI我的外部中断来自哪一个IO口
3.   设置NVIC, 配置中断分组,优先级和使能
4.   最后发送给CPU来处理这个中断



上面的是GPIO外部中断的流程, 也有其他外设中断,如串口通信等,这些不依赖于GPIO的会直接传输到NVIC里去





#### STM32 EXTI配置步骤

1.    使能GPIO时钟
2.   设置GPIO的输入模式
3.   使能AFIO/SYSCFG时钟
4.   设置EXTI和IO对应的关系
5.   设置EXTI屏蔽, 上/下沿
6.   设置NVIC: 分三步, 设置优先级分组, 设置优先级, 使能中断
7.   设置中断服务函数



![image-20241025150351890](img/image-20241025150351890.png)





### 编程实战

通过外部中断 控制一个等待亮灭

![image-20241025151115990](img/image-20241025151115990.png)



输入下拉是因为PA0是浮空的,需要给个稳定的低电压, 这样才能在keyUP按下的时候形成流动。 这时候就会导通产生一个上升沿, PE2-4同理, 是输入上拉,下降沿触发





中断服务函数调用逻辑

![image-20241025153538559](img/image-20241025153538559.png)

![image-20241025153705017](img/image-20241025153705017.png)

代码如下:

```c
#include "./BSP/EXTI/exti.h"
#include "./SYSTEM/delay/delay.h"

// 前六步初始化
void exti_init()
{
	GPIO_InitTypeDef gpio_init_struct;
	// enable the clock
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// config the struct
	gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
	gpio_init_struct.Pin = GPIO_PIN_4;
	gpio_init_struct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &gpio_init_struct);
	
	// config priority level
	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	
}

//第七步定义终端服务函数, 这个EXTI4_IRQHandler是系统已经定义好了的, 一旦EXTI4触发了终端服务程序,系统的NVIC就会自动的调用中服务函数,这里是调用的EXTI4_IRQHandler这个服务函数
void EXTI4_IRQHandler(void)
{
    // 这里进来后会统一进入外部中断服务处理函数
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

// rewrite the callback function


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 上面IRQHandler会自动调用这个Callback函数
	delay_ms(20);
	if(GPIO_Pin == GPIO_PIN_4)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		}
	}
}

```











## 数据通信

### 基础概念



#### 串行和并行通信

![image-20241025191528549](img/image-20241025191528549.png)





#### 单工/半双工/全双工通信

![image-20241025195325379](img/image-20241025195325379.png)





#### 同步/异步通信

![image-20241025195444593](img/image-20241025195444593.png)





#### 波特率

![image-20241025195544419](img/image-20241025195544419.png)

-   码元: 经过调制过后的信息, 1个码元可能有N个位, 1个码元能表示的信息种类为$2^n$种
-   比特率=波特率* $log_2M$= 波特率 * $N$;
-   在二进制系统中, N==1, 所以波特率等于比特率。日常电子产品基本都是二进制



#### 常见通信接口

![image-20241025200057148](img/image-20241025200057148.png)





### 串口(RS-232)

![image-20241025200827501](img/image-20241025200827501.png)

#### 电平特征

![image-20241025200921802](img/image-20241025200921802.png)

为了能够让RS-232和stm32或51来通信,需要一些电平转化芯片

#### 转化协议

![image-20241025201100603](img/image-20241025201100603.png)

![image-20241025201406416](img/image-20241025201406416.png)

#### 异步通信协议

![image-20241025201605120](img/image-20241025201605120.png)

有效数据位: 低位在前,高位在后





### USART

![image-20241025201854114](img/image-20241025201854114.png)

#### STM32 USART主要特性

![image-20241025202252865](img/image-20241025202252865.png)

![image-20241025202415220](img/image-20241025202415220.png)

上面的是F1的USART和UART的串口号,两个USART,三个UART



#### USART框图

![image-20241025203000739](img/image-20241025203000739.png)



![image-20241025203616313](img/image-20241025203616313.png)



简化图

![image-20241025204149440](img/image-20241025204149440.png)









#### F1的波特率设置

![image-20241025204303175](img/image-20241025204303175.png)



1.  如果使用USART1,那就是72MHz, 如果是其他串口,那么都是36MHz  (F1的情况下)



波特率设置:我们要设置的是DIV_Mantissa和DIV_Fraction, 这个的值为: DIV_Mantissa是USARTDIV的整数部分, 然后计算DIV_Fraction的值填入即可



USARTDIV 在波特率计算公式中, 波特率是用户设定的, 然后$f_{ck}$是串口的时钟频率, 这两个知道了, 就可以知道USARTDIV了



![image-20241025205121931](img/image-20241025205121931.png)





例子:

![image-20241025205206070](img/image-20241025205206070.png)

这里的fraction之所以乘以16,是由USARTDIV = mantissa + (fraction/16)这条公式得来的 

$$fraction = (USARTDIV-mantissa) * 16 $$

这里+0.5是为了浮点转整形后满足四舍五入



#### 波特率通用设置公式推到

![image-20241025210353595](img/image-20241025210353595.png)

>    公式推导公式过程:
>
>   $USART1->BRR  = (mantissa<<4) + fraction = mantissa * 16 + 0.0625 * 16 + 0.5 = USARTDIV * 16 + 0.5$











#### USART寄存器配置

##### CR1

![image-20241026111600992](img/image-20241026111600992.png)

CR1可以理解为就是一个使能寄存器,控制着一些总开关和发送接收开关

通常使用8个数据位,所以将12设置为0,选择8个数据位。

我们这里不采用校验, 所以禁止校验位





##### CR2

![image-20241026112412448](img/image-20241026112412448.png)

这个通常只需要配置对应的停止位即可, 告诉数据最后读到多少个位就算停止





##### CR3

![image-20241026112520853](img/image-20241026112520853.png)

配置通信模式: 双工/半双工

我们一般用不到半双工模式,所以位3位0即可





##### DR

![image-20241026113127412](img/image-20241026113127412.png)

用来存放收发数据的寄存器





##### SR

![image-20241026113251020](img/image-20241026113251020.png)

状态寄存器,用来判断当前的收发状态的

-   TC(发送完成)：告诉你数据是否发送完毕
-   RXNE(接收到数据)：告诉你是否收到新数据
-   就像快递跟踪系统，告诉你包裹的状态







##### 五个寄存器的理解

-   CR1就像打开微信的开关
-   CR2决定你用什么符号表示消息结束
-   CR3决定是语音还是文字聊天
-   DR就是你输入的实际内容
-   SR就是"对方正在输入"或"消息已送达"的提示





### HAL库回调机制

![image-20241026120323428](img/image-20241026120323428.png)

这里的PPP表示通用外设,可以是外设的名字



何时使用这个MspInit()

![image-20241026120841191](img/image-20241026120841191.png)





以USART为例:

这个就是可以基于基地址来判断我初始化哪个外设

![image-20241026120954375](img/image-20241026120954375.png)







USART中断的回调机制

![image-20241026181621289](img/image-20241026181621289.png)





![image-20241026181746001](img/image-20241026181746001.png)

![image-20241026182828541](img/image-20241026182828541.png)





### USART异步通信配置

![image-20241026183307538](img/image-20241026183307538.png)



#### HAL_UART_Init()

![image-20241026183342954](img/image-20241026183342954.png)

有HandleTypeDef的字眼的结构体,我们称之为句柄,通常用来配置一个外设的初始化属性的



这个关键结构体是UART_HandleTypeDef中的一个属性





#### HAL_UART_Receive_IT()

![image-20241026184223183](img/image-20241026184223183.png)

以_IT结尾的都是开启中断的函数, 接收需要中断提醒cpu,发送不用



#### HAL_UART_Transmit()

![image-20241026184358332](img/image-20241026184358332.png)

以阻塞的方式发送,发不完就阻塞,直至超时 









### 复用引脚

**复用引脚: 即多功能 I/O 引脚,允许一个引脚可以被配置为不同的功能**

判断引脚是不是复用引脚,只需要判断他的输入输出是否由GPIO控制

![image-20241027160444103](img/image-20241027160444103.png)

复用功能AF0-15的含义: 每个复用功能包含了不同外设的功能,比如AF1就是定时器1和定时器2的功能, AF7就是USART的通信功能,通过将对应的引脚设置为对应的AF功能,就可以让那个引脚发挥出对应的外设功能。这些需要通过设置AFRL或AFRH寄存器来定义



#### F1的IO引脚复用:

![image-20241027161942932](img/image-20241027161942932.png)

有部分IO是无法重映射的, 所以只能在功能设计上避免冲突



先查表->再决定配置寄存器的什么位置->打开对应的时钟和基础配置



#### F4以上的IO引脚复用:

![image-20241027162058570](img/image-20241027162058570.png)



![image-20241027162304751](img/image-20241027162304751.png)

![image-20241027162317032](img/image-20241027162317032.png)

![image-20241027162322559](img/image-20241027162322559.png)

假如我要用USART的功能,经过查表发现,是AF7的, 且用到引脚9和引脚10, 所以要将上面的AFRH9和10的寄存器位设置为0111







### 编程实战

通过串口接收或者发送一个字符。



我们向32发送一个字符, 之后32接收后触发中断,向我们发送回这个字符

设计思路:

1. USART函数初始化时设置接收逻辑, 接收到外部的信息就会自动产生一个中断, 这个中断会将这个消息存放到一个变量里面。定义一个标志位,改变这个标志位从而通知发送端
2. 发送端写到main函数里面,发现标志位被改了以后,我们就可以直接发送了数据了。



原理图:

![image-20241027164713195](img/image-20241027164713195.png)





设置UART的初始化,同时设置接收中断函数

```c
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"



uint8_t g_usart_rx_buf[USART_REC_LEN];
uint8_t g_usart_rx_flag = 0; // 串口接收到了数据的标志
UART_HandleTypeDef g_uart1_handle;  /* UART句柄 */


/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart_init(uint32_t baudrate)
{
    /*UART 初始化设置*/
    g_uart1_handle.Instance = USART1;                                       /* USART_UX */
    g_uart1_handle.Init.BaudRate = baudrate;                                  /* 波特率 */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&g_uart1_handle);                                           /* HAL_UART_Init()会使能UART1 */

    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE); 
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART1)                            /* 如果是串口1，进行串口1 MSP初始化 */
    {
        // enable the clock
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        // init IO
        
        gpio_init_struct.Pin = GPIO_PIN_9;                   /* LED0引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 推挽输出 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);       /* 初始化LED0引脚 */
        
        
        gpio_init_struct.Pin = GPIO_PIN_10;                   /* LED0引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;            /* 输入 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);       /* 初始化LED0引脚 */
        
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}



/**
 * @brief       串口1中断服务函数
 * @param       无
 * @retval      无
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&g_uart1_handle);   /* 为防止不断有中断到来,先禁用中断功能, 然后调用HAL库中断处理公用函数 */
}

/**
 * @brief       串口数据接收回调函数
                数据处理在这里进行
 * @param       huart:串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 设置接收标志 */
    g_usart_rx_flag = 1;
    
    /* 重新开启接收中断 */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, 1);
}


```



步骤:

1. 配置USART一些基础配置,如选择哪个串口, 波特率是多少, 停止位, 字长之类的
2. 配置USART初始化HAL_UART_Init(), 他会自动调用HAL_UART_MspInit(), 我们只需要配置这个MSPInit函数即可
   1. 打开USART的时钟和GPIO的时钟
   2. 对GPIO的引脚做设置, 分别对PA9和PA10做配置, PA9是TX口,发送的,所以是推挽复用。PA10是RX口,接收的,所以是复用输入
   3. 设置NVIC的中断优先级
3. 编写中断服务函数, USART1的终端服务函数在.s启动文件里面找,USART1_IRQHandler()
4. 编写UART1的接收中断回调处理函数HAL_UART_RxCpltCallback, 把接收标志位设置为1,表示我已接收到了数据,告诉主函数运行的发送程序要发送信息了



主函数:

```c
#include "./stm32f1xx_it.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"


int main(void)
{
    /* 初始化部分 */
    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);
    usart_init(115200);
    
    while (1)
    {
        if (g_usart_rx_flag == 1)
        {
            /* 发送数据回去 */
            HAL_UART_Transmit(&g_uart1_handle, g_rx_buffer, 1, 100);
            
            /* 发送换行 */
            uint8_t newline[] = "\r\n";
            HAL_UART_Transmit(&g_uart1_handle, newline, 2, 100);
            
            /* 清除标志 */
            g_usart_rx_flag = 0;
        }
        
        delay_ms(10);  // 适当延时
    }
}

```





关键函数:

1. HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE); 配置接收事件,如果收到信息后会存放在g_rx_buffer这个buffer池里面, 每次接收RXBUFFERSIZE
2. /* 重新开启接收中断 */
       HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, 1);
3. 注意点,每次触发HAL_UART_IRQHandler函数时,我们都会关闭这个Receive_IT的使能,所以在回调函数的时候要记得重新打开





## IWDG

独立看门狗: Independent watchdog



![image-20241028212705475](img/image-20241028212705475.png)

这个计数器的晶振的独立的, 电压来自V_dd, 只要供电就能用



![image-20241028213333564](img/image-20241028213333564.png)



### IWDG的作用

![image-20241028213427554](img/image-20241028213427554.png)

### 工作原理

![image-20241028213643986](img/image-20241028213643986.png)

![image-20241028213849112](img/image-20241028213849112.png)









