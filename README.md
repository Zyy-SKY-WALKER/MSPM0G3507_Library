![逐飞LOGO](https://images.gitee.com/uploads/images/2019/0924/114256_eaf16bad_1699060.png "逐飞科技logo 中.png")
# 逐飞科技MSPM0G3507开源库
#### 简介
该项目为逐飞科技基于德州仪器 LP_MSPM0G3507制作的MCU开源库。简化了部分库函数功能的使用步骤，便于使用MSPM0G3507参加竞赛以及进行产品开发。

#### 环境准备
1.  **MSPM0G3507硬件环境：** 
![逐飞科技MSPM0G3507核心板](https://gitee.com/seekfree/MSPM0G3507_Library/raw/master/assets/DesktopIMG20250606102544.png "逐飞科技MSPM0G3507核心板.jpg")
- MSPM0G3507核心板，[点击此处购买](https://item.taobao.com/item.htm?ft=t&id=940659357625)
2.  **软件开发环境：** 
（MDK ）
- MDK 推荐使用版本：MDK v5.37及以上。（5.26版本后加入了对DAP仿真器V2版本的支持，可以使用本公司DAP仿真器的WinUSB模式进行更高速的下载）
3.  **仿真器：** 
（DAP仿真器）

![逐飞科技DAP](https://gitee.com/seekfree/MSPM0G3507_Library/raw/master/assets/MSPM0G3507核心板拓展板详情页_18.jpg "逐飞科技DAP.jpg")

- DAP仿真器：推荐使用本公司DAP仿真器，双下载模式（有线下载、无线下载），可以在支持的环境下实现更高下载速度，[点击此处购买](https://item.taobao.com/item.htm?ft=t&id=583404964920)。
#### 使用说明

1.  **下载开源库：** 点击页面右侧的克隆/下载按钮，将工程文件保存到本地。您可以使用git克隆（Clone）或下载ZIP压缩包的方式来下载。推荐使用git将工程目录克隆到本地，这样可以使用git随时与我们的开源库保持同步。关于码云与git的使用教程可以参考以下链接 [https://gitee.com/help](https://gitee.com/help)。
2.  **打开工程：** 将下载好的工程文件夹打开（若下载的为ZIP文件，请先解压压缩包）。在打开工程前，请务必确保您的IDE满足环境准备章节的要求。否则可能出现打开工程时报错，提示丢失目录信息等问题。
- 若您使用的IDE为MDK，则工程文件保存在Project/MDK文件夹下。


#### 扩展功能引脚介绍

 **扩展板实物图** 
![逐飞科技MSPM0G3507扩展板](https://gitee.com/seekfree/MSPM0G3507_Library/raw/master/assets/DSC03369.png "逐飞科技MSPM0G3507扩展板.jpg")

 **扩展板引脚功能介绍图** 

 ![逐飞科技MSPM0G3507扩展板引脚功能介绍图](https://gitee.com/seekfree/MSPM0G3507_Library/raw/master/assets/cd2928baf9819e6da13e067cc969d75c.png "逐飞科技MSPM0G3507扩展板引脚功能介绍图.jpg")

 - 电池供电接口：推荐使用直流 7.2V - 26V的电池进行供电。
 -   电源主开关：当主板使用电池进行供电时，此开关为主板整体供电开关。
