---
title: DDNS
date: 2017-08-29 19:54:42
tags:
---

Background
---
**项目所用的服务器是基础电信线路，没有申请固定ip，需要使用ddns实时更新服务器ipv4地址。**  <!--more-->

Features
---
&emsp;&emsp;最新版本的花生壳已不再支持动态域名（ddns），仅支持内网穿透。<br>&emsp;&emsp;而花生壳之前发布的软件phddns2.0仍支持ddns，在[http://hsk.oray.com/download/](http://hsk.oray.com/download/)下载经典版即可。

&emsp;&emsp;在ubuntu下，使用dpkg安装***(sudo dpkg -i  [软件包名])***后，直接运行phddns即可进行交互式配置。服务器配置和网卡配置保持默认即可，花生壳官网的账号就是登陆所需的用户名密码。交互式配置完成后ddns服务就开始运行。***务必记住配置文件所在位置，以便之后修改，这个位置可以在先前的交互式配置中设置***。服务正常运行后，即可把它加入系统启动项中。

&emsp;&emsp;在ubuntu下，自订服务脚本需要自定init info，否则会报错。

&emsp;&emsp;以ubuntu16.04LTS为例，在/etc/init.d中新建一个shell脚本<br>
```
sudo vi phddns-autorun.sh
```
按i进入插入模式，</br>
然后录入以下文本，#!/bin/sh以下是init info部分
```
#!/bin/sh
### BEGIN INIT INFO
# Provides:          phddns_oray_ddns
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: phddnsd
# Description:       phddns daemon
### END INIT INFO
phddns -d
exit 0
```
phddns -d 以守护程序方式运行，在前面的交互式配置之后，该方式会默认使用先前设置好的配置文件。

&emsp;&emsp;录入完成后按ESC然后输入:wq!保存缓冲区并退出。然后把该脚本设置为默认情况下的开机启动项。<br>

```
sudo update-rc.d phddns-autorun.sh defaults
```

&emsp;&emsp;设置成功后，为该服务器设置的动态域名可到[花生壳控制台](https://b.oray.com/)登录查看，默认情况下所有的壳域名均可作为动态域名使用。

&emsp;&emsp;花生壳官网同时提供了可免费申请的ddns服务SDK，可用于服务器集群的ddns配置。



