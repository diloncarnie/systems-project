#ifndef MPU9250_H
#define MPU9250_H
#include <cstddef>

namespace MPU9250
{

    class MPU9250
    {
    public:
        enum GyroRange
        {
            GYRO_RANGE_250DPS,
            GYRO_RANGE_500DPS,
            GYRO_RANGE_1000DPS,
            GYRO_RANGE_2000DPS
        };
        enum AccelRange
        {
            ACCEL_RANGE_2G,
            ACCEL_RANGE_4G,
            ACCEL_RANGE_8G,
            ACCEL_RANGE_16G
        };
        enum DlpfBandwidth
        {
            DLPF_BANDWIDTH_184HZ,
            DLPF_BANDWIDTH_92HZ,
            DLPF_BANDWIDTH_41HZ,
            DLPF_BANDWIDTH_20HZ,
            DLPF_BANDWIDTH_10HZ,
            DLPF_BANDWIDTH_5HZ
        };
        enum LpAccelOdr
        {
            LP_ACCEL_ODR_0_24HZ = 0,
            LP_ACCEL_ODR_0_49HZ = 1,
            LP_ACCEL_ODR_0_98HZ = 2,
            LP_ACCEL_ODR_1_95HZ = 3,
            LP_ACCEL_ODR_3_91HZ = 4,
            LP_ACCEL_ODR_7_81HZ = 5,
            LP_ACCEL_ODR_15_63HZ = 6,
            LP_ACCEL_ODR_31_25HZ = 7,
            LP_ACCEL_ODR_62_50HZ = 8,
            LP_ACCEL_ODR_125HZ = 9,
            LP_ACCEL_ODR_250HZ = 10,
            LP_ACCEL_ODR_500HZ = 11
        };

        MPU9250(int bus, unsigned int address);

        int begin();
        int setAccelRange(AccelRange range);
        int setGyroRange(GyroRange range);
        int setDlpfBandwidth(DlpfBandwidth bandwidth);
        int setSrd(unsigned int srd);
        int enableDataReadyInterrupt();
        int disableDataReadyInterrupt();
        int enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr);
        int readSensor();
        float getAccelX_mss();
        float getAccelY_mss();
        float getAccelZ_mss();
        float getGyroX_rads();
        float getGyroY_rads();
        float getGyroZ_rads();
        float getMagX_uT();
        float getMagY_uT();
        float getMagZ_uT();
        float getTemperature_C();

        int calibrateGyro();
        float getGyroBiasX_rads();
        float getGyroBiasY_rads();
        float getGyroBiasZ_rads();
        void setGyroBiasX_rads(float bias);
        void setGyroBiasY_rads(float bias);
        void setGyroBiasZ_rads(float bias);
        int calibrateAccel();
        float getAccelBiasX_mss();
        float getAccelScaleFactorX();
        float getAccelBiasY_mss();
        float getAccelScaleFactorY();
        float getAccelBiasZ_mss();
        float getAccelScaleFactorZ();
        void setAccelCalX(float bias, float scaleFactor);
        void setAccelCalY(float bias, float scaleFactor);
        void setAccelCalZ(float bias, float scaleFactor);
        int calibrateMag();
        float getMagBiasX_uT();
        float getMagScaleFactorX();
        float getMagBiasY_uT();
        float getMagScaleFactorY();
        float getMagBiasZ_uT();
        float getMagScaleFactorZ();
        void setMagCalX(float bias, float scaleFactor);
        void setMagCalY(float bias, float scaleFactor);
        void setMagCalZ(float bias, float scaleFactor);

    protected:
        // i2c
        int i2c_file_;
        unsigned int _address;
        const unsigned int _i2cRate = 400000; // 400 kHz
        size_t _numBytes;                 // number of bytes received from I2C
        unsigned int _csPin;
        bool _useSPI;
        bool _useSPIHS;
        const unsigned int SPI_READ = 0x80;
        const unsigned int SPI_LS_CLOCK = 1000000;  // 1 MHz
        const unsigned int SPI_HS_CLOCK = 15000000; // 15 MHz
        // track success of interacting with sensor
        int _status;
        // buffer for reading from sensor
        unsigned int _buffer[21];
        // data counts
        int _axcounts, _aycounts, _azcounts;
        int _gxcounts, _gycounts, _gzcounts;
        int _hxcounts, _hycounts, _hzcounts;
        int _tcounts;
        // data buffer
        float _ax, _ay, _az;
        float _gx, _gy, _gz;
        float _hx, _hy, _hz;
        float _t;
        // wake on motion
        unsigned int _womThreshold;
        // scale factors
        float _accelScale;
        float _gyroScale;
        float _magScaleX, _magScaleY, _magScaleZ;
        const float _tempScale = 333.87f;
        const float _tempOffset = 21.0f;
        // configuration
        AccelRange _accelRange;
        GyroRange _gyroRange;
        DlpfBandwidth _bandwidth;
        unsigned int _srd;
        // gyro bias estimation
        size_t _numSamples = 100;
        double _gxbD, _gybD, _gzbD;
        float _gxb, _gyb, _gzb;
        // accel bias and scale factor estimation
        double _axbD, _aybD, _azbD;
        float _axmax, _aymax, _azmax;
        float _axmin, _aymin, _azmin;
        float _axb, _ayb, _azb;
        float _axs = 1.0f;
        float _ays = 1.0f;
        float _azs = 1.0f;
        // magnetometer bias and scale factor estimation
        unsigned int _maxCounts = 1000;
        float _deltaThresh = 0.3f;
        unsigned int _coeff = 8;
        unsigned int _counter;
        float _framedelta, _delta;
        float _hxfilt, _hyfilt, _hzfilt;
        float _hxmax, _hymax, _hzmax;
        float _hxmin, _hymin, _hzmin;
        float _hxb, _hyb, _hzb;
        float _hxs = 1.0f;
        float _hys = 1.0f;
        float _hzs = 1.0f;
        float _avgs;
        // transformation matrix
        /* transform the accel and gyro axes to match the magnetometer axes */
        const int tX[3] = {0, 1, 0};
        const int tY[3] = {1, 0, 0};
        const int tZ[3] = {0, 0, -1};
        // constants
        const float G = 9.807f;
        const float _d2r = 3.14159265359f / 180.0f;
        // MPU9250 registers
        const unsigned int ACCEL_OUT = 0x3B;
        const unsigned int GYRO_OUT = 0x43;
        const unsigned int TEMP_OUT = 0x41;
        const unsigned int EXT_SENS_DATA_00 = 0x49;
        const unsigned int ACCEL_CONFIG = 0x1C;
        const unsigned int ACCEL_FS_SEL_2G = 0x00;
        const unsigned int ACCEL_FS_SEL_4G = 0x08;
        const unsigned int ACCEL_FS_SEL_8G = 0x10;
        const unsigned int ACCEL_FS_SEL_16G = 0x18;
        const unsigned int GYRO_CONFIG = 0x1B;
        const unsigned int GYRO_FS_SEL_250DPS = 0x00;
        const unsigned int GYRO_FS_SEL_500DPS = 0x08;
        const unsigned int GYRO_FS_SEL_1000DPS = 0x10;
        const unsigned int GYRO_FS_SEL_2000DPS = 0x18;
        const unsigned int ACCEL_CONFIG2 = 0x1D;
        const unsigned int ACCEL_DLPF_184 = 0x01;
        const unsigned int ACCEL_DLPF_92 = 0x02;
        const unsigned int ACCEL_DLPF_41 = 0x03;
        const unsigned int ACCEL_DLPF_20 = 0x04;
        const unsigned int ACCEL_DLPF_10 = 0x05;
        const unsigned int ACCEL_DLPF_5 = 0x06;
        const unsigned int CONFIG = 0x1A;
        const unsigned int GYRO_DLPF_184 = 0x01;
        const unsigned int GYRO_DLPF_92 = 0x02;
        const unsigned int GYRO_DLPF_41 = 0x03;
        const unsigned int GYRO_DLPF_20 = 0x04;
        const unsigned int GYRO_DLPF_10 = 0x05;
        const unsigned int GYRO_DLPF_5 = 0x06;
        const unsigned int SMPDIV = 0x19;
        const unsigned int INT_PIN_CFG = 0x37;
        const unsigned int INT_ENABLE = 0x38;
        const unsigned int INT_DISABLE = 0x00;
        const unsigned int INT_PULSE_50US = 0x00;
        const unsigned int INT_WOM_EN = 0x40;
        const unsigned int INT_RAW_RDY_EN = 0x01;
        const unsigned int PWR_MGMNT_1 = 0x6B;
        const unsigned int PWR_CYCLE = 0x20;
        const unsigned int PWR_RESET = 0x80;
        const unsigned int CLOCK_SEL_PLL = 0x01;
        const unsigned int PWR_MGMNT_2 = 0x6C;
        const unsigned int SEN_ENABLE = 0x00;
        const unsigned int DIS_GYRO = 0x07;
        const unsigned int USER_CTRL = 0x6A;
        const unsigned int I2C_MST_EN = 0x20;
        const unsigned int I2C_MST_CLK = 0x0D;
        const unsigned int I2C_MST_CTRL = 0x24;
        const unsigned int I2C_SLV0_ADDR = 0x25;
        const unsigned int I2C_SLV0_REG = 0x26;
        const unsigned int I2C_SLV0_DO = 0x63;
        const unsigned int I2C_SLV0_CTRL = 0x27;
        const unsigned int I2C_SLV0_EN = 0x80;
        const unsigned int I2C_READ_FLAG = 0x80;
        const unsigned int MOT_DETECT_CTRL = 0x69;
        const unsigned int ACCEL_INTEL_EN = 0x80;
        const unsigned int ACCEL_INTEL_MODE = 0x40;
        const unsigned int LP_ACCEL_ODR = 0x1E;
        const unsigned int WOM_THR = 0x1F;
        const unsigned int WHO_AM_I = 0x75;
        const unsigned int FIFO_EN = 0x23;
        const unsigned int FIFO_TEMP = 0x80;
        const unsigned int FIFO_GYRO = 0x70;
        const unsigned int FIFO_ACCEL = 0x08;
        const unsigned int FIFO_MAG = 0x01;
        const unsigned int FIFO_COUNT = 0x72;
        const unsigned int FIFO_READ = 0x74;
        // AK8963 registers
        const unsigned int AK8963_I2C_ADDR = 0x0C;
        const unsigned int AK8963_HXL = 0x03;
        const unsigned int AK8963_CNTL1 = 0x0A;
        const unsigned int AK8963_PWR_DOWN = 0x00;
        const unsigned int AK8963_CNT_MEAS1 = 0x12;
        const unsigned int AK8963_CNT_MEAS2 = 0x16;
        const unsigned int AK8963_FUSE_ROM = 0x0F;
        const unsigned int AK8963_CNTL2 = 0x0B;
        const unsigned int AK8963_RESET = 0x01;
        const unsigned int AK8963_ASA = 0x10;
        const unsigned int AK8963_WHO_AM_I = 0x00;
        // private functions
        int writeRegister(unsigned int subAddress, unsigned int data);
        int readRegister(unsigned int subAddress, unsigned int count, unsigned int *dest);
        int writeAK8963Register(unsigned int subAddress, unsigned int data);
        int readAK8963Registers(unsigned int subAddress, unsigned int count, unsigned int *dest);
        int whoAmI();
        int whoAmIAK8963();
    };

}
#endif // MPU9250_H