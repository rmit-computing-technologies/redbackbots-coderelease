/**
 * Interface to a camera of the NAO V6.
 *
 * Modified for RedBackBots from the
 * original implementation by B-Human
 *
 * @author Colin Graf (B-human)
 * @author Thomas Röfer (B-human)
 * @author RedBackBots
 */

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#include "perception/vision/camera/CameraDefinitions.hpp"
#include "perception/vision/camera/NaoCameraDefinitions.hpp"
#include "perception/vision/camera/NaoCameraV6.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "utils/Logger.hpp"

// from https://github.com/aldebaran/linux-aldebaran/blob/sbr/v4.4.86-rt99-baytrail/drivers/media/i2c/soc_camera/ov5640.c

#define AEC_STABLE_RANGE_HIGH           0x3A0F
#define AEC_STABLE_RANGE_LOW            0x3A10
#define AEC_HYSTERESIS_RANGE_HIGH       0x3A1B
#define AEC_HYSTERESIS_RANGE_LOW        0x3A1E

// The maximum gain and exposure values for a v6 camera.
#define V6_MAX_GAIN 1023
#define V6_MAX_EXPOSURE_ABSOLUTE 1048575
#define V6_MAX_BRIGHTNESS 255
#define V6_MAX_CONTRAST 255
#define V6_MAX_SATURATION 255

// On the V6, this is needed
#define EXTENSION_UNIT_ID 3

/**
 * sets all bytes of x to 0
 * @param x the variable to clear
 */
#define CLEAR(x) memset(&(x), 0, sizeof(x))

NaoCameraV6::NaoCameraV6(const std::string device, std::string camera, int width, int height,
                         const CameraSettings &settings) : camera(camera),
                                                           WIDTH(width),
                                                           HEIGHT(height)
{
    resetRequired = (fd = open(device.c_str(), O_RDWR | O_NONBLOCK)) == -1;
    if (resetRequired) {
        llog(ERROR) << camera << ": file handle open failed" << std::endl;
    }
    usleep(30000); // Experimental: Add delay between opening and using camera device
    resetRequired = resetRequired || !setImageFormat() || !setFrameRate(1, 30) || !mapBuffers() || !queueBuffers();
    if (resetRequired) {
        llog(ERROR) << camera << ": configuring camera structures failed" << std::endl;
    }
    if (!resetRequired) {
        resetRequired = !startCapturing();
        writeCameraSettings(settings, camera == "camera.top");
        // readCameraSettings();
    }
    if (resetRequired) {
        llog(ERROR) << camera << ": Setting up NaoCameraV6 failed!" << std::endl;
    }
}

NaoCameraV6::~NaoCameraV6()
{
    stopCapturing();
    unmapBuffers();

    close(fd);
}

bool NaoCameraV6::captureNew(int timeout)
{
    // requeue the buffer of the last captured image which is obsolete now
    if (currentBuf) {
        if (ioctl(fd, VIDIOC_QBUF, currentBuf) == -1) {
            return false;
        }
    }

    pollfd pollfd = {fd, POLLIN | POLLPRI, 0};
    int polled = poll(&pollfd, 1, timeout);
    if (polled < 0)
    {
        llog(ERROR) << camera << "camera : Cannot poll. Reason: " << strerror(errno) << std::endl;
        ::abort();
    }
    else if (polled == 0)
    {
        llog(ERROR) << camera << "camera : " << timeout << " ms passed and there's still no image to read from the camera. Terminating." << std::endl;
        return false;
    }
    else if (pollfd.revents & (POLLERR | POLLNVAL))
    {
        llog(ERROR) << camera << "camera : Polling failed." << std::endl;
        return false;
    }

    // dequeue a frame buffer (this call blocks when there is no new image available) */
    v4l2_buffer lastBuf;
    bool firstAttempt = true;
    while (ioctl(fd, VIDIOC_DQBUF, buf) == 0)
    {
        if (firstAttempt)
            firstAttempt = false;
        else if (ioctl(fd, VIDIOC_QBUF, &lastBuf) != 0)
            return false;
        lastBuf = *buf;
    }

    if (errno != EAGAIN)
    {
        llog(ERROR) << "VIDIOC_DQBUF failed: " << strerror(errno) << std::endl;
        return false;
    }
    else
    {
        currentBuf = buf;
        timestamp = static_cast<unsigned long long>(currentBuf->timestamp.tv_sec) * 1000000ll + currentBuf->timestamp.tv_usec;

        if (first)
        {
            first = false;
            llog(INFO) << camera << "camera is working" << std::endl;
        }

        return true;
    }
}

void NaoCameraV6::releaseImage()
{
    if (currentBuf)
    {
        if (ioctl(fd, VIDIOC_QBUF, currentBuf) == -1)
        {
            llog(ERROR) << camera << ": Releasing image failed!" << std::endl;
            resetRequired = true;
        }
        currentBuf = nullptr;
    }
}

const uint8_t *NaoCameraV6::getImage() const
{
    return currentBuf ? mem[currentBuf->index] : nullptr;
}

bool NaoCameraV6::hasImage()
{
    return !!currentBuf;
}

unsigned long long NaoCameraV6::getTimestamp() const {
    if (!currentBuf) {
        return 0;
    }
    return timestamp;
}

float NaoCameraV6::getFrameRate() const {
    return 1.f / 30.f;
}

bool NaoCameraV6::setFrameRate(unsigned numerator, unsigned denominator) {
    // set frame rate
    v4l2_streamparm fps;
    memset(&fps, 0, sizeof(v4l2_streamparm));
    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &fps)) {
        return false;
    }
    fps.parm.capture.timeperframe.numerator = numerator;
    fps.parm.capture.timeperframe.denominator = denominator;
    return ioctl(fd, VIDIOC_S_PARM, &fps) != -1;
}

void NaoCameraV6::resetCamera() {
    llog(INFO) << "(static) reset camera" << std::endl;
    usleep(100000);
    int fileDescriptor = -1;
    if (openI2CDevice(fileDescriptor)) {
        unsigned char config;
        if (i2cReadByteData(fileDescriptor, 0x41, 0x3, config) && (config & 0xc) != 0) {
            i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0x0});
            i2cWriteBlockData(fileDescriptor, 0x41, 0x3, {0xf3});
        }
        i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0x0});
        i2cWriteBlockData(fileDescriptor, 0x41, 0x1, {0xc});
    }
    if (fileDescriptor != -1) {
        close(fileDescriptor);
    }
    sleep(2);
}

bool NaoCameraV6::openI2CDevice(int &fileDescriptor) {
    return (fileDescriptor = open("/dev/i2c-head", O_RDWR | O_NONBLOCK)) >= 0;
}

bool NaoCameraV6::i2cReadWriteAccess(int fileDescriptor, unsigned char readWrite, unsigned char command, unsigned char size, i2c_smbus_data &data) {
    struct i2c_smbus_ioctl_data ioctlData;
    ioctlData.read_write = readWrite;
    ioctlData.command = command;
    ioctlData.size = size;
    ioctlData.data = &data;
    return ioctl(fileDescriptor, I2C_SMBUS, &ioctlData) >= 0;
}

bool NaoCameraV6::i2cWriteBlockData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, std::vector<unsigned char> data) {
    if (ioctl(fileDescriptor, I2C_SLAVE, deviceAddress) < 0) {
        return false;
    }

    i2c_smbus_data writeData;
    writeData.block[0] = static_cast<unsigned char>(std::min(32, static_cast<int>(data.size())));
    for (int i = 1; i <= writeData.block[0]; ++i)
    {
        writeData.block[i] = data[i - 1];
    }
    return i2cReadWriteAccess(fileDescriptor, I2C_SMBUS_WRITE, dataAddress, I2C_SMBUS_I2C_BLOCK_DATA, writeData);
    ;
}

bool NaoCameraV6::i2cReadByteData(int fileDescriptor, unsigned char deviceAddress, unsigned char dataAddress, unsigned char &res)
{
    if (ioctl(fileDescriptor, I2C_SLAVE, deviceAddress) < 0)
        return false;

    i2c_smbus_data readData;
    if (!i2cReadWriteAccess(fileDescriptor, I2C_SMBUS_READ, dataAddress, I2C_SMBUS_BYTE_DATA, readData)) {
        return false;
    }
    res = readData.block[0];
    return true;
}

void NaoCameraV6::writeCameraSettings(const CameraSettings settings, bool top) {
//   use `v4l2-ctl --list-ctrls-menus --device=/dev/video0` to see what options are available
//   use `v4l2-ctl --set-ctrl=<ctrl>=<val>[,<ctrl>=<val>...] --device=/dev/video0` to set ctrls, even while redbackbots & offnao are running

    // have to do it in this order when setting focus to be near
    setControl(V4L2_CID_FOCUS_ABSOLUTE, settings.focusAbsolute);
    setControl(V4L2_CID_FOCUS_AUTO, 1);
    setControl(V4L2_CID_FOCUS_AUTO, settings.autoFocus);
    // 0 is auto, 1 is manual. Need to toggle or it won't work sometimes.
    setControl(V4L2_CID_EXPOSURE_AUTO, settings.exposureAuto);
    setControl(V4L2_CID_EXPOSURE_AUTO, !settings.exposureAuto);
    if(settings.exposureAuto) {
        setAutoExposureTarget(fd, settings.aeTargetExposure);
    }

    setControl(V4L2_CID_BRIGHTNESS, (settings.brightness+1));// %
                                                //V6_MAX_BRIGHTNESS);
    setControl(V4L2_CID_BRIGHTNESS, settings.brightness);
    setControl(V4L2_CID_CONTRAST, (settings.contrast+1));// % V6_MAX_CONTRAST);
    setControl(V4L2_CID_CONTRAST, settings.contrast);
    setControl(V4L2_CID_SATURATION, (settings.saturation+1));// %
                                                //V6_MAX_SATURATION);
    setControl(V4L2_CID_SATURATION, settings.saturation);

    if (!settings.exposureAuto) {
        // Manual exposure does not work correctly unless the gain and
        // exposure are set to different values when activated.
        setControl(V4L2_CID_GAIN, (settings.gain+1) % V6_MAX_GAIN);
        setControl(V4L2_CID_GAIN, settings.gain);
        setControl(V4L2_CID_EXPOSURE_ABSOLUTE, (settings.exposure+1) % V6_MAX_EXPOSURE_ABSOLUTE);
        setControl(V4L2_CID_EXPOSURE_ABSOLUTE, settings.exposure);
    }


    // Flip Camera Horizontally
    uint16_t valHFlip = static_cast<uint16_t>(settings.hflip);
    uint16_t curHFlip = ~valHFlip; // so it's initialized
    do {
        // Set and Check setting
        queryXU(true, 12, &valHFlip, 2);
        queryXU(false, 12, &curHFlip, 2);
        //   set_uvc_xu(fd, EXTENSION_UNIT_ID, 12, 2, &valHFlip);
        //   get_uvc_xu(fd, EXTENSION_UNIT_ID, 12, 2, &curHFlip);
    } while (curHFlip != valHFlip);

    // Flip Camera Vertically
    uint16_t valVFlip = static_cast<uint16_t>(settings.vflip);
    uint16_t curVFlip = ~valVFlip; // so it's initialized
    do {
        queryXU(true, 13, &valVFlip, 2);
        queryXU(false, 13, &curVFlip, 2);
        //   set_uvc_xu(fd, EXTENSION_UNIT_ID, 13, 2, &valVFlip);
        //   get_uvc_xu(fd, EXTENSION_UNIT_ID, 13, 2, &curVFlip);
    } while (curVFlip != valVFlip);

   // For now, this aeUseWeightTable only works for top camera (dont' use for bottom)
    if (settings.aeUseWeightTable){
        unsigned width = top ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
        unsigned height = top ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
        // Refer to https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf for details of following array
        uint8_t windowVals[17] = {
            1, // enable
            0, // upper four bits of x coordinate auto exposure window start
            0, // lower eight bits of x coordinate auto exposure window start
            0, // upper three bits of y coordinate auto exposure window start
            0, // lower eight bits of y coordinate auto exposure window start
            (uint8_t) (width >> 8u), // upper four bits of x coordinate auto exposure window end
            (uint8_t) width,  // lower eight bits of x coordinate auto exposure window end
            (uint8_t) (height >> 8u), // upper three bits of y coordinate auto exposure window end
            (uint8_t) height,  // upper eight bits of y coordinate auto exposure window end
            0u << 4u | 0u,
            0u << 4u | 0u,
            0u << 4u | 0u,
            0u << 4u | 0u,
            10u << 4u | 10u,
            10u << 4u | 10u,
            10u << 4u | 10u,
            10u << 4u | 10u};

        uint8_t  curWindowVals[17];
        for (int wv = 0; wv < 17; ++wv) {
            curWindowVals[wv] = ~windowVals[wv];
        }
        do {
            queryXU(true, 9, windowVals, 17);
            queryXU(false, 9, curWindowVals, 17);
            //  set_uvc_xu(fd, EXTENSION_UNIT_ID, 9, 17, windowVals);
            //  get_uvc_xu(fd, EXTENSION_UNIT_ID, 9, 17, curWindowVals);
        } while (memcmp(curWindowVals, windowVals, 17));
    }
}

void NaoCameraV6::changeResolution(int width, int height) {
    llog(TRACE) << "change resolution" << std::endl;

    resetRequired = resetRequired || !stopCapturing();
    unmapBuffers();

    WIDTH = width;
    HEIGHT = height;

    resetRequired = resetRequired || !setImageFormat() || !mapBuffers() || !queueBuffers() || !startCapturing();
    if (resetRequired) {
        llog(ERROR) << "Changing camera resolution failed!" << std::endl;
    }
}

bool NaoCameraV6::setImageFormat() {
    // set format
    v4l2_format fmt;
    memset(&fmt, 0, sizeof(v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.sizeimage = WIDTH * HEIGHT * 2;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt)) {
        return false;
    }

    return true;
}

bool NaoCameraV6::mapBuffers() {
    // request buffers
    v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(v4l2_requestbuffers));
    rb.count = frameBufferCount;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
    {
        return false;
    }

    // map or prepare the buffers
    buf = static_cast<v4l2_buffer *>(calloc(1, sizeof(v4l2_buffer)));
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    for (unsigned i = 0; i < frameBufferCount; ++i)
    {
        buf->index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, buf) == -1)
        {
            return false;
        }
        memLength[i] = buf->length;
        mem[i] = reinterpret_cast<uint8_t*>(
            mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset));
    }
    return true;
}

void NaoCameraV6::unmapBuffers() {
    // unmap buffers
    for (unsigned i = 0; i < frameBufferCount; ++i)
    {
        munmap(mem[i], memLength[i]);
        mem[i] = nullptr;
        memLength[i] = 0;
    }

    free(buf);
    currentBuf = buf = nullptr;
}

bool NaoCameraV6::queueBuffers() {
    for (unsigned i = 0; i < frameBufferCount; ++i) {
        buf->index = i;
        if (ioctl(fd, VIDIOC_QBUF, buf) == -1) {
            return false;
        }
    }
    return true;
}

bool NaoCameraV6::startCapturing() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(fd, VIDIOC_STREAMON, &type) != -1;
}

bool NaoCameraV6::stopCapturing() {
    llog(INFO) << camera << ": Capturing stopping" << std::endl;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(fd, VIDIOC_STREAMOFF, &type) != -1;
}

bool NaoCameraV6::queryXU(bool set, unsigned char control, void *value, unsigned short size) const {
    uvc_xu_control_query xu;
    xu.unit = EXTENSION_UNIT_ID;
    xu.selector = control;
    xu.query = set ? UVC_SET_CUR : UVC_GET_CUR;
    xu.size = size;
    xu.data = static_cast<__u8 *>(value);
    return !ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

bool NaoCameraV6::setRegister(unsigned short address, unsigned short value) const {
    unsigned char bytes[5];
    bytes[0] = 1;
    bytes[1] = static_cast<unsigned char>(address >> 8);
    bytes[2] = static_cast<unsigned char>(address & 0xff);
    bytes[3] = static_cast<unsigned char>(value >> 8);
    bytes[4] = static_cast<unsigned char>(value & 0xff);
    return setXU(14, bytes);
}

bool NaoCameraV6::getRegister(unsigned short address, unsigned short &value) const {
    unsigned char bytes[5] = {0, static_cast<unsigned char>(address >> 8),
                              static_cast<unsigned char>(address & 0xff)};
    if (setXU(14, bytes))
    {
        usleep(100000);
        if (getXU(14, bytes))
        {
            value = static_cast<unsigned short>(bytes[3] << 8 | bytes[4]);
            return true;
        }
    }
    return false;
}

bool NaoCameraV6::setControl(const uint32_t controlId, const int32_t controlValue) {
    struct v4l2_control control;
    CLEAR(control);
    if (controlId < V4L2_CID_BASE) {
        control.id = V4L2_CID_BASE + controlId;
    } else {
        control.id    = controlId;
    }
    control.value = controlValue;

    struct v4l2_control current;
    CLEAR(current);
    current.id = control.id;
    ioctl (fd, VIDIOC_G_CTRL, &current);
    std::cerr << fd << " - Setting id " << controlId <<
        " \033[36m" << CAMERA_CONTROL_VALUE_MAP[controlId] <<
        ": " << controlValue << "\033[0m...";
    int i = 0;
    while (current.value != controlValue) {
        ioctl(fd, VIDIOC_S_CTRL, &control);
        ioctl (fd, VIDIOC_G_CTRL, &current);
        //cout << "setting id " << control.id << "to " << controlValue << std::endl;
        if (++i == 1000000) {
        llog (ERROR) << "Nao camera set control is stuck!  This head may be dead!" << std::endl;
        std::cerr << "\tstuck!\n"
                        "\n"
                        "Even a reboot doesn't fix this...  Not sure what to do...\n"
                        "\n";
        }
    }
    std::cerr << "\tOK!\n";

    return true;
}

/*
WARNING: There are issues with reading and writing to the camera. We think data
3 is always ignored. It also appears that whatever you do, a read will just read
the value of the last register written to.
*/

int set_uvc_xu_v6(
   int device_fd,
   uint8_t extension_unit_id,
   uint8_t control_selector,
   uint16_t size,
   void *data)
{
   struct uvc_xu_control_query query = {
      extension_unit_id,
      control_selector,
      UVC_SET_CUR,
      size,
      (__u8 *) data
   };
   return ioctl(device_fd, UVCIOC_CTRL_QUERY, &query);
}

int writeRegister_v6(int fd, uint16_t addr, uint8_t val) {
    uint8_t data[5];
    data[0] = 1;
    data[1] = static_cast<uint8_t>(addr >> 8);
    data[2] = static_cast<uint8_t>(addr & 0xff);
    data[3] = 0x00;
    data[4] = val;
    int output = set_uvc_xu_v6(fd, EXTENSION_UNIT_ID, 14, 5, data);
    if (output < 0) {
        llog(ERROR) << "writeRegister-set_uvc_xu failed:\t" << strerror(errno) << "\n";
    }
    return output;
}

void NaoCameraV6::setAutoExposureTarget(int fd, uint8_t high) {
   const uint8_t low = static_cast<const uint8_t>(high * 0x30 / 0x48); // scale based on defaults in kernel source
   writeRegister_v6(fd, AEC_STABLE_RANGE_HIGH, high);
   writeRegister_v6(fd, AEC_STABLE_RANGE_LOW, low);
   writeRegister_v6(fd, AEC_HYSTERESIS_RANGE_HIGH, high);
   writeRegister_v6(fd, AEC_HYSTERESIS_RANGE_LOW, low);
   // uvc_xu seems to read from the last address we wrote, so write to this one again so we can read it later
   writeRegister_v6(fd, AEC_STABLE_RANGE_HIGH, high);
}

/** 
 * B-Human camera settings system - kept here while we decide if we want to move to this
 */

// NaoCameraV6::CameraSettingsCollection::CameraSettingsCollection()
// {
//   settings[CameraSettings::autoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000, 0, 3);
//   settings[CameraSettings::autoExposureBrightness] = V4L2Setting(V4L2_CID_BRIGHTNESS, -1000, -255, 255);
//   settings[CameraSettings::exposure] = V4L2Setting(V4L2_CID_EXPOSURE_ABSOLUTE, -1000, 0, 1048575, CameraSettings::autoExposure, 1);
//   settings[CameraSettings::gain] = V4L2Setting(V4L2_CID_GAIN, -1000, 0, 1023, CameraSettings::autoExposure, 1);
//   settings[CameraSettings::autoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, -1000, 0, 1);
//   settings[CameraSettings::autoFocus] = V4L2Setting(V4L2_CID_FOCUS_AUTO, -1000, 0, 1);
//   settings[CameraSettings::focus] = V4L2Setting(V4L2_CID_FOCUS_ABSOLUTE, -1000, 0, 250, CameraSettings::autoFocus);
//   settings[CameraSettings::autoHue] = V4L2Setting(V4L2_CID_HUE_AUTO, -1000, 0, 1);
//   settings[CameraSettings::hue] = V4L2Setting(V4L2_CID_HUE, -1000, -180, 180, CameraSettings::autoHue);
//   settings[CameraSettings::saturation] = V4L2Setting(V4L2_CID_SATURATION, -1000, 0, 255);
//   settings[CameraSettings::contrast] = V4L2Setting(V4L2_CID_CONTRAST, -1000, 0, 255);
//   settings[CameraSettings::sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, -1000, 0, 9);
//   settings[CameraSettings::redGain] = V4L2Setting(-0xe3400, -1000, 0, 4095, CameraSettings::autoWhiteBalance);
//   settings[CameraSettings::greenGain] = V4L2Setting(-0xe3402, -1000, 0, 4095, CameraSettings::autoWhiteBalance);
//   settings[CameraSettings::blueGain] = V4L2Setting(-0xe3404, -1000, 0, 4095, CameraSettings::autoWhiteBalance);
//   for(size_t i = 0; i < sizeOfAutoExposureWeightTable; ++i)
//     autoExposureWeightTable[i] = V4L2Setting(0, -1000, 0, AutoExposureWeightTable::maxWeight);
// }

// bool NaoCameraV6::CameraSettingsCollection::operator==(const CameraSettingsCollection& other) const
// {
//   FOREACH_ENUM(CameraSettings::CameraSetting, setting)
//   {
//     if(settings[setting] != other.settings[setting])
//       return false;
//   }
//   for(size_t i = 0; i < autoExposureWeightTable.size(); ++i)
//   {
//     if(autoExposureWeightTable[i] != other.autoExposureWeightTable[i])
//       return false;
//   }
//   return true;
// }

// bool NaoCameraV6::CameraSettingsCollection::operator!=(const CameraSettingsCollection& other) const
// {
//   return !(*this == other);
// }

// NaoCameraV6::CameraSettingsSpecial::CameraSettingsSpecial() :
//   verticalFlip(-0xc0000, 0, 0, 1),
//   horizontalFlip(-0xd0000, 0, 0, 1)
// {}


// void NaoCameraV6::writeCameraSettings()
// {
    //   const auto oldSettings = appliedSettings.settings;
    //   FOREACH_ENUM(CameraSettings::CameraSetting, settingName)
    //   {
    //     V4L2Setting& currentSetting = settings.settings[settingName];
    //     V4L2Setting& appliedSetting = appliedSettings.settings[settingName];

    //     if(timestamp == 0)
    //     {
    //       if(currentSetting.notChangableWhile != CameraSettings::numOfCameraSettings &&
    //          settings.settings[currentSetting.notChangableWhile].value ^ currentSetting.invert)
    //         continue;
    //     }
    //     else
    //     {
    //       if(currentSetting.notChangableWhile != CameraSettings::numOfCameraSettings)
    //       {
    //         const bool nowActive = settings.settings[currentSetting.notChangableWhile].value ^ currentSetting.invert;
    //         const bool oldActive = oldSettings[currentSetting.notChangableWhile].value ^ currentSetting.invert;
    //         if(nowActive || (!oldActive && currentSetting.value == appliedSetting.value))
    //           continue;
    //       }
    //       else if(currentSetting.value == appliedSetting.value)
    //         continue;
    //     }

    //     if(!setControlSetting(currentSetting)) {
    //       OUTPUT_WARNING("NaoCameraV6: Setting camera control " << TypeRegistry::getEnumName(settingName) << " failed for value: " << currentSetting.value);
    //     } else {
    //       appliedSetting.value = currentSetting.value;
    //     }
    //   }

    //   for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i)
    //   {
    //     V4L2Setting& currentTableEntry = settings.autoExposureWeightTable[i];
    //     V4L2Setting& appliedTableEntry = appliedSettings.autoExposureWeightTable[i];
    //     if(timestamp != 0 && currentTableEntry.value == appliedTableEntry.value) {}
    //       continue;
    //     }

    //     // Set all weights at once
    //     unsigned char value[17] =
    //     {
    //       1, 0, 0, 0, 0,
    //       static_cast<unsigned char>(WIDTH >> 8), static_cast<unsigned char>(WIDTH & 8),
    //       static_cast<unsigned char>(HEIGHT >> 8), static_cast<unsigned char>(HEIGHT & 8)
    //     };

    //     // Set weights in value to set
    //     for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i)
    //     {
    //       V4L2Setting& currentTableEntry = settings.autoExposureWeightTable[i];
    //       currentTableEntry.enforceBounds();
    //       ASSERT(9 + i / 2 < sizeof(value));
    //       value[9 + i / 2] |= currentTableEntry.value << (i & 1) * 4;
    //     }

    //     // Use extension unit to set exposure weight table
    //     if(setXU(9, value))
    //       appliedSettings.autoExposureWeightTable = settings.autoExposureWeightTable;
    //     else
    //       OUTPUT_ERROR("NaoCameraV6: setting auto exposure weight table failed");

    //     break; // All weights were set, no need to search for more differences
    //   }
// }

// void NaoCameraV6::readCameraSettings()
// {
    // for(V4L2Setting& setting : appliedSettings.settings) {
    //   getControlSetting(setting);
    // }
    // unsigned char value[17];
    // if(getXU(9, value)) {
    //   for(size_t i = 0; i < CameraSettingsCollection::sizeOfAutoExposureWeightTable; ++i) {
    //     settings.autoExposureWeightTable[i].value = (value[9 + i / 2] >> (i & 1) * 4) & 0x0f;
    //   }
    // } else {
    //   llog(ERROR) << "NaoCameraV6: reading auto exposure weight table failed" << std::endl;
    // }
// }

// void NaoCameraV6::doAutoWhiteBalance() {
    // appliedSettings.settings[CameraSettings::autoWhiteBalance].value = 1;
    // setControlSetting(appliedSettings.settings[CameraSettings::autoWhiteBalance]);
    // usleep(100);
    // unsigned short hiRedGain, loRedGain, hiGreenGain, loGreenGain, hiBlueGain, loBlueGain;
    // if(getRegister(0x3400, hiRedGain) && getRegister(0x3401, loRedGain)
    //    && getRegister(0x3402, hiGreenGain) && getRegister(0x3403, loGreenGain)
    //    && getRegister(0x3404, hiBlueGain) && getRegister(0x3405, loBlueGain))
    // {
    //   appliedSettings.settings[CameraSettings::redGain].value
    //     = settings.settings[CameraSettings::redGain].value = hiRedGain << 8 | loRedGain;
    //   appliedSettings.settings[CameraSettings::greenGain].value
    //     = settings.settings[CameraSettings::greenGain].value = hiGreenGain << 8 | loGreenGain;
    //   appliedSettings.settings[CameraSettings::blueGain].value
    //     = settings.settings[CameraSettings::blueGain].value = hiBlueGain << 8 | loBlueGain;
    //   OUTPUT_TEXT("New white balance is RGB = (" << settings.settings[CameraSettings::redGain].value << ", "
    //                                              << settings.settings[CameraSettings::greenGain].value << ", "
    //                                              << settings.settings[CameraSettings::blueGain].value << ")");
    //   appliedSettings.settings[CameraSettings::autoWhiteBalance].value = 0;
    //   setControlSetting(appliedSettings.settings[CameraSettings::autoWhiteBalance]);
    // }
// }

// bool NaoCameraV6::checkSettingsAvailability() {
    // bool status = true;
    // for(V4L2Setting& setting : appliedSettings.settings) {
    //   status = status && checkV4L2Setting(setting);
    // }
    // return status;
// }

// bool NaoCameraV6::checkV4L2Setting(V4L2Setting &setting) const {
//     if (setting.command < 0)
//     {
//         return true;
//     }

//     v4l2_queryctrl queryctrl;
//     queryctrl.id = setting.command;
//     if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
//     {
//         llog(ERROR) << "ioctl to query setting failed for camera setting " << setting.command << std::endl;
//         return false;
//     }
//     if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
//     {
//         llog(ERROR) << "Camera setting " << setting.command << " is disabled" << std::endl;
//     }
//     if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
//     {
//         llog(ERROR) << "Camera setting " << setting.command << "is unsupported " << std::endl;
//     }
//     setting.setCameraBounds(queryctrl.minimum, queryctrl.maximum);
//     return true;
// }

// bool NaoCameraV6::getControlSetting(V4L2Setting &setting)
// {
//     if (setting.command >= 0)
//     {
//         v4l2_control control_s;
//         control_s.id = setting.command;
//         if (ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
//         {
//             llog(ERROR) << "NaoCameraV6: Retrieving camera setting " << setting.command << " failed" << std::endl;
//             return false;
//         }
//         setting.value = control_s.value;
//     }
//     return true;
// }

// bool NaoCameraV6::setControlSetting(V4L2Setting &setting)
// {
//     setting.enforceBounds();
//     if (setting.command < 0) // command for extension unit?
//     {
//         if ((-setting.command) >> 16 == 14) // set register
//         {
//             unsigned short address = static_cast<unsigned short>(-setting.command & 0xffff);
//             if (!setRegister(address, static_cast<unsigned char>(setting.value >> 8)))
//             {
//                 llog(ERROR) << "NaoCameraV6: Setting register " << address << " failed." << std::endl;
//                 return false;
//             }
//             if (!setRegister(address + 1, static_cast<unsigned char>(setting.value & 0xff)))
//             {
//                 llog(ERROR) << "NaoCameraV6: Setting register " << address << " failed." << std::endl;
//                 return false;
//             }
//         }
//         else if (!setXU(static_cast<unsigned char>((-setting.command) >> 16), static_cast<unsigned short>(setting.value)))
//         {
//             llog(ERROR) << "NaoCameraV6: Extension unit selector " << ((-setting.command) >> 16) << " failed." << std::endl;
//             return false;
//         }
//     }
//     else
//     {
//         v4l2_control control_s;
//         control_s.id = setting.command;
//         control_s.value = setting.value;

//         const int ret = ioctl(fd, VIDIOC_S_CTRL, &control_s);
//         if (ret < 0)
//         {
//             llog(ERROR) << "NaoCameraV6: Setting value ID: " << setting.command << " failed. VIDIOC_S_CTRL return value is " << ret << std::endl;
//             return false;
//         }
//     }
//     return true;
// }

// NaoCameraV6::V4L2Setting::V4L2Setting(int command, int value, int min, int max, CameraSettings notChangableWhile, int invert) : command(command), value(value), notChangableWhile(notChangableWhile), invert(invert), min(min), max(max)
// {
// }

// bool NaoCameraV6::V4L2Setting::operator==(const V4L2Setting &other) const {
//     return command == other.command && value == other.value;
// }

// bool NaoCameraV6::V4L2Setting::operator!=(const V4L2Setting &other) const {
//     return !(*this == other);
// }

// void NaoCameraV6::V4L2Setting::enforceBounds() {
//     if (value < min) {
//         value = min;
//     } else if (value > max) {
//         value = max;
//     }
// }

// void NaoCameraV6::V4L2Setting::setCameraBounds(int camMin, int camMax) {
//     min = std::max(camMin, min);
//     max = std::min(camMax, max);
// }