#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <sstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm = *std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") 
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

//#define DEBUG

#define LOGE(fmt,args...) fprintf(stderr,"%s %s:%d " fmt "\n",getCurrentTimestamp().c_str(),__FUNCTION__,__LINE__,##args)
#define LOGI(fmt,args...) fprintf(stdout,"%s %s:%d " fmt "\n",getCurrentTimestamp().c_str(),__FUNCTION__,__LINE__,##args)
#ifdef DEBUG
#define LOGV(fmt,args...) fprintf(stdout,"%s %s:%d " fmt "\n",getCurrentTimestamp().c_str(),__FUNCTION__,__LINE__,##args)
#define LOGD(fmt,args...) fprintf(stdout,"%s %s:%d " fmt "\n",getCurrentTimestamp().c_str(),__FUNCTION__,__LINE__,##args)
#else
#define LOGV(fmt,args...)
#define LOGD(fmt,args...)
#endif

volatile sig_atomic_t keepRunning = 1;

void signalHandler(int signum) {
    if (signum == SIGINT) {
        LOGD("\n捕获到 Ctrl+C, 退出程序中...");
        keepRunning = 0;
        signal(SIGINT, SIG_DFL);
    }
}

template<typename T>
void setBit0(T& data,int bit) {
    T mask = ~(1 << bit);
    data = data & mask;
}

template<typename T>
void setBit1(T& data,int bit) {
    T mask = 1 << bit;
    data = data | mask;
}

template<typename T>
void setBit(T&data,int bit, int value) {
    if(value) {
        setBit1(data,bit);
    } else {
        setBit0(data,bit);
    }
}

// 使用原生I2C接口写入16位数据
int i2c_write_register(int fd, uint8_t addr, uint8_t reg, uint16_t data) {
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;
    
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data & 0xFF;       // 小端模式，低字节在前
    buf[2] = (data >> 8) & 0xFF; // 高字节在后
    
    msg.addr = addr;
    msg.flags = 0; // 写标志
    msg.len = 3;
    msg.buf = buf;
    
    msgset.msgs = &msg;
    msgset.nmsgs = 1;
    
    if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C write failed");
        return -1;
    }
    return 0;
}

// 使用原生I2C接口读取16位数据
int i2c_read_register(int fd, uint8_t addr, uint8_t reg, uint16_t *value) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t reg_buf[1];
    uint8_t data_buf[2];
    
    // 第一步：发送要读取的寄存器地址
    reg_buf[0] = reg;
    msgs[0].addr = addr;
    msgs[0].flags = 0; // 写标志
    msgs[0].len = 1;
    msgs[0].buf = reg_buf;
    
    // 第二步：读取数据
    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD; // 读标志
    msgs[1].len = 2;
    msgs[1].buf = data_buf;
    
    msgset.msgs = msgs;
    msgset.nmsgs = 2;
    
    if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C read failed");
        return -1;
    }
    
    // VEML7700使用小端字节序
    *value = (data_buf[1] << 8) | data_buf[0];
    return 0;
}

enum ALS_GAIN {
    GAIN_X_2, //01
    GAIN_X_1, //00
    GAIN_X_1_4, //11
    GAIN_X_1_8, //10
};

enum ALS_INTEGRATION {
    IT_800_MS, //0011
    IT_400_MS, //0010
    IT_200_MS, //0001
    IT_100_MS, //0000
    IT_50_MS,  //1000
    IT_25_MS,  //1100
};

float getResolution(uint16_t config) {
    static float resolution_table[6][4]={
        {0.0042f,0.0084f,0.0336f,0.0672f},
        {0.0084f,0.0168f,0.0672f,0.1344f},
        {0.0168f,0.0336f,0.1344f,0.2688f},
        {0.0336f,0.0672f,0.2688f,0.5376f},
        {0.0672f,0.1344f,0.5376f,1.0752f},
        {0.1344f,0.2688f,1.0752f,2.1504f},
    };

    ALS_GAIN gain = GAIN_X_1;
    switch ((config >> 11) & 0x03) {
        case 0x00: gain = GAIN_X_1;break;
        case 0x01: gain = GAIN_X_2;break;
        case 0x02: gain = GAIN_X_1_8;break;
        case 0x03: gain = GAIN_X_1_4;break;
    }

    ALS_INTEGRATION itms = IT_100_MS;
    switch ((config >> 6) & 0x0F) {
        case 0x0C: itms = IT_25_MS; break;
        case 0x08: itms = IT_50_MS; break;
        case 0x00: itms = IT_100_MS; break;
        case 0x01: itms = IT_200_MS; break;
        case 0x02: itms = IT_400_MS; break;
        case 0x03: itms = IT_800_MS; break;
    }
    return resolution_table[itms][gain];
}

class Veml7700 {
public:
#define ALS_CONFIG_REG 0x00
#define ALS_DATA_REG 0x04
    Veml7700(){
        int bus = 2;
        char filename[20];
        snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
        
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            perror("Failed to open I2C bus");
            return ;
        }
        if(ioctl(fd, I2C_SLAVE_FORCE, 0x10) < 0){
            perror("Failed to select device");
            return;
        }
    };
    ~Veml7700(){
        if(fd >= 0){
            close(fd);
            fd = -1;
        }
    };
    bool powerOn(){
        uint16_t config = 0;
        setBit0(config,0);
        setBit0(config,1); //disable interrupt
        setBit0(config,2); //reserved
        setBit0(config,3); //reserved
        LOGV("wirte config %02x",config);
        if(i2c_write_register(fd,0x10,ALS_CONFIG_REG,config)){
            LOGE("i2c write config failed");
            return false;
        }
        LOGI("wirted config %02x",config);
        return true;
    }
    bool shutdown(){
        uint16_t config = 0;
        setBit(config,0,1);
        LOGV("wirte config %02x",config);
        if(i2c_write_register(fd,0x10,ALS_CONFIG_REG,config)){
            LOGE("i2c write config failed");
            return false;
        }
        LOGV("wirted config %02x",config);
        return true;
    }

    bool setGain(ALS_GAIN gain){
        uint16_t config = 0;
        if(i2c_read_register(fd,0x10,ALS_CONFIG_REG,&config)){
            LOGE("read config failed");
            return false;
        }
        switch(gain){
            case GAIN_X_1:
                setBit(config,12,0);
                setBit(config,11,0);
                break;
            case GAIN_X_2:
                setBit(config,12,0);
                setBit(config,11,1);
                break;
            case GAIN_X_1_4:
                setBit(config,12,1);
                setBit(config,11,1);
                break;
            case GAIN_X_1_8:
                setBit(config,12,1);
                setBit(config,11,0);
                break;
            default:
                setBit(config,12,0);
                setBit(config,11,0);
                break;
        }
        LOGV("wirte config %02x",config);
        if(i2c_write_register(fd,0x10,ALS_CONFIG_REG,config)){
            LOGE("i2c write config failed");
            return false;
        }
        LOGV("wirted config %02x",config);
        return true;
    }
    bool setIntegration(ALS_INTEGRATION it){
        uint16_t config = 0;
        if(i2c_read_register(fd,0x10,ALS_CONFIG_REG,&config)){
            LOGE("read config failed");
            return false;
        }
        switch(it){
            case IT_25_MS:  //1100
                setBit(config,9,1);
                setBit(config,8,1);
                setBit(config,7,0);
                setBit(config,6,0);
                break;
            case IT_50_MS:  //1000
                setBit(config,9,1);
                setBit(config,8,0);
                setBit(config,7,0);
                setBit(config,6,0);
                break;
            IT_100_MS: //0000
                setBit(config,9,0);
                setBit(config,8,0);
                setBit(config,7,0);
                setBit(config,6,0);
                break;
            case IT_200_MS: //0001
                setBit(config,9,0);
                setBit(config,8,0);
                setBit(config,7,0);
                setBit(config,6,1);
                break;
            case IT_400_MS: //0010
                setBit(config,9,0);
                setBit(config,8,0);
                setBit(config,7,1);
                setBit(config,6,0);
                break;
            case IT_800_MS: //0011
                setBit(config,9,0);
                setBit(config,8,0);
                setBit(config,7,1);
                setBit(config,6,1);
                break;
            default:
                setBit(config,9,0);
                setBit(config,8,0);
                setBit(config,7,0);
                setBit(config,6,0);
                break;
        }
        LOGV("wirte config %02x",config);
        if(i2c_write_register(fd,0x10,ALS_CONFIG_REG,config)){
            LOGE("i2c write config failed");
            return false;
        }
        LOGV("wirted config %02x",config);
        return true;
    }

    bool getLux(float& lux){
        uint16_t alsdata = 0;
        if(i2c_read_register(fd,0x10,ALS_DATA_REG,&alsdata)){
            LOGE("read alsdata failed");
            return false;
        }
        LOGD("read alsdata:%02x",alsdata);
        uint16_t config = 0;
        if(i2c_read_register(fd,0x10,ALS_CONFIG_REG,&config)){
            LOGE("read config failed");
            return false;
        }
        LOGD("read conf:%02x",config);

        float resolution = getResolution(config);
        LOGI("resolution=%f alsdata=%04x config=%04x",resolution,alsdata,config);
        lux = resolution * alsdata;
        return true;
    }
private:
    int fd = -1;
    uint8_t mConfig = 0;
#undef ALS_CONFIG_REG
#undef ALS_DATA_REG
};

int Veml7700Test() {
    Veml7700 veml7700;

    if(!veml7700.powerOn()) {
        LOGE("power on failed");
        return -1;
    }
    veml7700.setGain(GAIN_X_1_8);
    veml7700.setIntegration(IT_25_MS);
    while (keepRunning) {
        float lux = 0 ;
        if(veml7700.getLux(lux)){
            LOGI("Lux: %f",lux);
        }
        usleep(200000);
    }
    return 0;
}

int main (int argc,char** argv) {
    signal(SIGINT, signalHandler);
    Veml7700Test();
    LOGD("goodbye");
    return 0;
}