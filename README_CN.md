# FM Transmitter
[English](https://github.com/markondej/fm_transmitter/blob/master/README.md) | [简体中文](https://github.com/markondej/fm_transmitter/blob/master/README_CN.md)

把树莓派当做FM发射器，可工作在任何树莓派上。

仅需要获得一个收音机, 把一段20 - 40厘米的线连接到树莓派的GPIO4端口上 (GPIO的第7个)充当天线, 以及准备好广播的心态。

这个项目使用通用时钟输出来产生FM信号， 它基于[PiFM project](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter). 最初由[Oliver Mattos and Oskar Weigl](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter)。
## 安装和使用
为了使用该软件，你需要构建可执行的文件，首先，安装依赖项。
```bash
sudo apt-get update
sudo apt-get install make build-essential
```
根据操作系统(例如 Ubuntu Server 20.10) 的不同，可能还要需要安装Broadcom库。
```bash
sudo apt-get install libraspberrypi-dev
```
安装完依赖后，克隆这个库然后使用'make'命令来生成可执行文件。
```bash
git clone https://github.com/markondej/fm_transmitter
cd fm_transmitter
make
```
成功构建后，你就可以执行"fm_transmitter"来开始广播了。
```bash
sudo ./fm_transmitter -f 102.0 acoustic_guitar_duet.wav
```
如何？:
* -f 频率 - 以MHz为单位来指定频率，如果无效将默认为100.0MHz
* acoustic_guitar_duet.wav - 示例音乐，可以换成你自己的

其他选项:
* -d DMA通道 - 指定要使用的DMA通道（默认0），使用255来禁用DMA，将会使用CPU代替。
* -b 带宽 - 以kHz的单位来指定的带宽，默认情况为100KHz
* -r - 循环播放

传输开始后，只要把你的接收器调到你所选的频率上即可听到播放的声音。
### 树莓派4
在树莓派4上，有些内置硬件可能会对软件造成干扰，从而使其无法在所有标准FM频率上传输，对此，我们建议：
1. 在编译可执行文件时添加"GPIO21=1"的参数（PIN40）
```bash
make GPIO21=1
```
2. 将ARM核心频率调节器的设置更改为“performance”，或将ARM最大和最小频率更改为同一数字 (参阅: https://www.raspberrypi.org/forums/viewtopic.php?t=152692 ).
```bash
echo "performance"| sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```
3. 在传输时使用较低的FM频率（低于93 MHz）
### 支持的音频格式
你可以直接传输未压缩的WAV(.wav)文件，也可以从stdin中读取音频，例如：
```bash
sudo apt-get install sox
sox acoustic_guitar_duet.wav -r 22050 -c 1 -b 16 -t wav - | sudo ./fm_transmitter -f 100.6 -
```
请注意，仅支持未压缩的WAV文件，如果报告"corrupted data"，请尝试转换文件，例如使用FFMPEG
```bash
ffmpeg -i not_wav_song.webm -f wav -bitexact -acodec pcm_s16le -ar 22050 -ac 1 song.wav
sudo ./fm_transmitter -f 100.6 song.wav
```
或使用SOX
```bash
sudo apt-get install sox libsox-fmt-mp3
sox my-audio.mp3 -r 22050 -c 1 -b 16 -t wav my-converted-audio.wav
sudo ./fm_transmitter -f 100.6 my-converted-audio.wav
```
### 麦克风支持
为了使用麦克风实时输入，请使用`arecord` 命令，例如:
```bash
arecord -D hw:1,0 -c1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
如果性能下降，请使用 ```plughw:1,0```代替 ```hw:1,0``` ，比如说:
```bash
arecord -D plughw:1,0 -c1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
## 法律说明
请记住，你未经许可的在你的国家/地区内的某些频率上进行传输可能是非法的。
## 新的功能
* DMA外设支持
* 允许自定义频率，带宽
* 适用于所有树莓派型号
* 可读取单声道和立体声文件
* 可从stdin中读取数据

随附带的示例音频由 [graham_makes](https://freesound.org/people/graham_makes/sounds/449409/) 创建，发布在 [freesound.org](https://freesound.org/)上。
