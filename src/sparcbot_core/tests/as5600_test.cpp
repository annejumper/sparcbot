#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <chrono>

constexpr const char* I2C_DEVICE = "/dev/i2c-1";
constexpr uint8_t AS5600_ADDR = 0x36;

constexpr uint8_t RAW_ANGLE_HIGH = 0x0C;
constexpr uint8_t RAW_ANGLE_LOW  = 0x0D;


uint8_t readRegister(int fd, uint8_t reg)
{
    // Tell AS5600 which register we want
    if (write(fd, &reg, 1) != 1)
    {
        throw std::runtime_error("Failed to write register address");
    }

    uint8_t value;

    // Read one byte from that register
    if (read(fd, &value, 1) != 1)
    {
        throw std::runtime_error("Failed to read register");
    }

    return value;
}


uint16_t readRawAngle(int fd)
{
    uint8_t high = readRegister(fd, RAW_ANGLE_HIGH);
    uint8_t low  = readRegister(fd, RAW_ANGLE_LOW);

    // AS5600 angle is only 12 bits.
    //
    // high byte:
    // xxxx HHHH
    //
    // low byte:
    // LLLLLLLL
    //
    // Combined:
    //
    // HHHH LLLLLLLL
    //
    // = 12 bits

    uint16_t raw =
        (static_cast<uint16_t>(high & 0x0F) << 8)
        | low;

    return raw;
}


int main()
{
    int fd = open(I2C_DEVICE, O_RDWR);

    if (fd < 0)
    {
        std::cerr << "Could not open " << I2C_DEVICE << "\n";
        return 1;
    }

    // Select AS5600 as the device we want to communicate with
    if (ioctl(fd, I2C_SLAVE, AS5600_ADDR) < 0)
    {
        std::cerr << "Could not select AS5600 at address 0x36\n";
        close(fd);
        return 1;
    }

    std::cout << "Reading AS5600 raw angle...\n";

    try
    {
        while (true)
        {
            uint16_t raw = readRawAngle(fd);

            std::cout
                << "Raw angle: "
                << std::setw(4)
                << raw
                << "\n";

            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "I2C error: " << e.what() << "\n";
    }

    close(fd);

    return 0;
}