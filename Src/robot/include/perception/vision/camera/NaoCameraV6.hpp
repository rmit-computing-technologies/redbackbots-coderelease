/**
 * @file Platform/Nao/NaoCamera.h
 *
 * Modified for RedBackBots from the
 * original implementation by B-Human (2021 code release)
 *
 * @author Colin Graf (B-human)
 * @author Thomas Röfer (B-human)
 * @author RedBackBots
 */

#pragma once

#include <array>
#include <limits>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string>
#include <vector>

#include "types/CameraSettings.hpp"

/**
 * @class NaoCameraV6
 * Interface to a camera of the NAO
 */
class NaoCameraV6 {
public:
    bool resetRequired = false;

    /**
     * @enum CameraPosition
     * Enum representing the possible sources of an image.
     */
    // enum CameraPosition
    // {
    //     top = 0,
    //     bottom = 1,
    //     unknown = 2,
    // };
    // static std::string cameraPositionAsString(CameraPosition &cp)
    // {
    //     if (cp == CameraPosition::top)
    //     {
    //         return "top";
    //     }
    //     else if (cp == CameraPosition::bottom)
    //     {
    //         return "bottom";
    //     }
    //     return "unknown";
    // };

    /**
     * Constructor.
     * @param device The name of the camera device (e.g. /dev/video0).
     * @param camera 'camera.top' or 'camera.bottom' as a human readable string
     * @param width The width of the camera image in pixels. V4L only allows certain values (e.g. 320 or 640).
     * @param height The height of the camera image in pixels. V4L only allows certain values (e.g. 240 or 480).
     * @param settings The camera settings.
     */
    NaoCameraV6(const std::string device, std::string camera, int width, int height, const CameraSettings &settings);
    ~NaoCameraV6();

    void changeResolution(int width, int height);

    /**
     * The method blocks till a new image arrives.
     * @param timeout The maximum waiting time
     * @return true (except a not manageable exception occurs)
     */
    bool captureNew(int timeout);

    /**
     * Releases an image that has been captured. That way the buffer can be used to capture another image
     */
    void releaseImage();

    /**
     * The last captured image.
     * @return The image data buffer.
     */
    const uint8_t *getImage() const;

    /**
     * Whether an image has been captured.
     * @return true if there is one
     */
    bool hasImage();

    /**
     * Timestamp of the last captured image in µs.
     * @return The timestamp.
     */
    unsigned long long getTimestamp() const;

    /**
     * Returns the frame rate used by the camera
     * @return The frame rate in frames per second
     */
    float getFrameRate() const;

    /**
     * Set frame rate in frames per 10 seconds.
     */
    bool setFrameRate(unsigned numerator = 1, unsigned denominator = 30);

    /**
     * Unconditional write of the camera settings to the camera
     */
    void writeCameraSettings(const CameraSettings settings, bool top);

    /**
     * Set a camera register.
     * @param address The address of the register.
     * @param value The value to set. Although the command could set two bytes,
     *              actual tests showed that only a single byte is set.
     * @return Was the call successful?
     */
    bool setRegister(unsigned short address, unsigned short value) const;

    /**
     * Get the current value of a register.
     * @param address The address of the register.
     * @param value The value returned. Although the command could return two bytes,
     *              actual tests showed that only a single byte is read.
     * @return Could the register be read?
     */
    bool getRegister(unsigned short address, unsigned short &value) const;

    static void resetCamera();
    static bool openI2CDevice(int &fileDescriptor);
    static bool i2cReadWriteAccess(int fileDescriptor, unsigned char readWrite, unsigned char command, unsigned char size, i2c_smbus_data &data);
    static bool i2cWriteBlockData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, std::vector<unsigned char> data);
    static bool i2cReadByteData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, unsigned char &res);

    /**
     * Write the given control setting to the camera
     */
    bool setControl(const uint32_t id, const int32_t value);
    
    static void setAutoExposureTarget(int fd, uint8_t high);

private:

    // The camera accessed by this driver as a human readable string
    std::string camera;

    static constexpr unsigned frameBufferCount = 3; /**< Amount of available frame buffers. */

    unsigned WIDTH;                           /**< The width of the yuv 422 image */
    unsigned HEIGHT;                          /**< The height of the yuv 422 image */
    int fd;                                   /**< The file descriptor for the video device. */
    // void *mem[frameBufferCount];              /**< Frame buffer addresses. */
    uint8_t *mem[frameBufferCount];              /**< Frame buffer addresses. */
    int memLength[frameBufferCount];          /**< The length of each frame buffer. */
    struct v4l2_buffer *buf = nullptr;        /**< Reusable parameter struct for some ioctl calls. */
    struct v4l2_buffer *currentBuf = nullptr; /**< The last dequeued frame buffer. */
    bool first = true;                        /**< First image grabbed? */
    unsigned long long timestamp = 0;         /**< Timestamp of the last captured image in microseconds. */

    bool setImageFormat();

    bool mapBuffers();
    void unmapBuffers();
    bool queueBuffers();

    bool startCapturing();
    bool stopCapturing();

    /**
     * Query a UVC control.
     * @param set Set value (instead of reading it)?
     * @param control The number of the control.
     * @param value Address of the value to set or get.
     * @param size Size of the value to set or get in bytes.
     * @return Did the call succeed?
     */
    bool queryXU(bool set, unsigned char control, void *value, unsigned short size) const;

    /**
     * Set a UVC control.
     * @param control The number of the control to set.
     * @param value The value to set.
     * @return Did the call succeed?
     */
    template <typename T>
    bool setXU(unsigned char control, const T &value) const
    {
        return queryXU(true, control, const_cast<T *>(&value), static_cast<unsigned short>(sizeof(T)));
    }

    /**
     * Get a UVC control.
     * @param control The number of the control to get.
     * @param value The value that is filled with the result.
     * @return Did the call succeed?
     */
    template <typename T>
    bool getXU(unsigned char control, const T &value) const
    {
        return queryXU(false, control, const_cast<T *>(&value), static_cast<unsigned short>(sizeof(T)));
    }


    // B-Human camera settings system - kept here while we decide if we want to move to this
    // void readCameraSettings();
    // void doAutoWhiteBalance();
    // bool checkV4L2Setting(V4L2Setting &setting) const;
    // bool checkSettingsAvailability();
    // bool getControlSetting(V4L2Setting &setting);
    // bool setControlSetting(V4L2Setting &setting);
    // CameraSettingsCollection settings; /**< The camera control settings. */
    // CameraSettingsCollection appliedSettings; /**< The camera settings that are known to be applied. */
    // CameraSettingsSpecial specialSettings; /**< Special settings that are only set */
    // class V4L2Setting
    // {
    // public:
    //     int command = 0;
    //     int value = 0;

    //     CameraSettings notChangableWhile;
    //     int invert = true;

    //     V4L2Setting() = default;
    //     V4L2Setting(int command, int value, int min, int max, CameraSettings notChangableWhile = CameraSettings(), int invert = 0);

    //     bool operator==(const V4L2Setting &other) const;
    //     bool operator!=(const V4L2Setting &other) const;

    //     void enforceBounds();
    //     void setCameraBounds(int camMin, int camMax);

    // private:
    //     int min = std::numeric_limits<int>::min();
    //     int max = std::numeric_limits<int>::max();
    // };

    // struct CameraSettingsCollection
    // {
    //   std::array<V4L2Setting, CameraSettings::numOfCameraSettings> settings;
    //   static constexpr size_t sizeOfAutoExposureWeightTable = AutoExposureWeightTable::width * AutoExposureWeightTable::height;
    //   std::array<V4L2Setting, sizeOfAutoExposureWeightTable> autoExposureWeightTable;

    //   CameraSettingsCollection();

    //   bool operator==(const CameraSettingsCollection& other) const;
    //   bool operator!=(const CameraSettingsCollection& other) const;
    // };

    // struct CameraSettingsSpecial
    // {
    //   V4L2Setting verticalFlip;
    //   V4L2Setting horizontalFlip;

    //   CameraSettingsSpecial();
    // };
};
