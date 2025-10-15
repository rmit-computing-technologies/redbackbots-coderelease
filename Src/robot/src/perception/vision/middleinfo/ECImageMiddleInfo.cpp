/**
 * @file ECImageMiddleInfo.cpp
 * 
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 * 
 */
#include "perception/vision/middleinfo/ECImageMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"

#include "utils/debug/Assert.hpp"
#include "utils/Logger.hpp"

#if !defined __arm64__ && !defined __aarch64__

ECImageMiddleInfo::ECImageMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
  Detector("ECImageMiddleInfo")
{
    configure(blackboard);
    this->jitRuntime = jitRuntime;

    llog(INFO) << NDEBUG_LOGSYMB << "ECImageMiddleInfo loaded" << std::endl;
}

ECImageMiddleInfo::~ECImageMiddleInfo() {
    if(eFunc) {
        this->jitRuntime->release(eFunc);
    }
    if(ecFunc) {
        this->jitRuntime->release(ecFunc);
    }
}

void ECImageMiddleInfo::configure(Blackboard* blackboard) {

}

void ECImageMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
  // TODO - reset the middle info for ECImage
}

void ECImageMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void ECImageMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera must run first, as the bottom camera is only computed if the top camera boundary can't be projected
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // TODO (TW): decide if lower field boundary is actually needed
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void ECImageMiddleInfo::detect_(CameraInfo::Camera whichCamera,
                                const VisionInfoIn* info_in, 
                                VisionInfoMiddle* info_middle, 
                                VisionInfoOut* info_out) {

    // llog(INFO) << NDEBUG_LOGSYMB << "Detect with " << CameraInfo::enumCameraToString(whichCamera) << std::endl;
    
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    const CameraImage* cameraImage = info_in->image[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];

    ecImage.grayscaled.setResolution(cameraInfo.width, cameraInfo.height);
    ecImage.saturated.setResolution(cameraInfo.width, cameraInfo.height);
    ecImage.hued.setResolution(cameraInfo.width, cameraInfo.height);
    ecImage.blueChromaticity.setResolution(cameraInfo.width / 2, cameraInfo.height / 2);
    ecImage.redChromaticity.setResolution(cameraInfo.width / 2, cameraInfo.height / 2);

    if(cameraImage->timestamp > 10 && static_cast<int>(cameraImage->width) == cameraInfo.width / 2) {
        // if(disableColor) {
        //   if(!eFunc) {
        //     compileE();
        //   }
        //   eFunc(cameraInfo.width * cameraInfo.height / 16, (*cameraImage)[0], ecImage.grayscaled[0]);
        // }
        // else {
        //   if(!ecFunc) {
        //     compileEC();
        //   }
        //   ecFunc(cameraInfo.width * cameraInfo.height / 16, (*cameraImage)[0], ecImage.grayscaled[0], ecImage.saturated[0], ecImage.hued[0]);
        // }

        if(!ecFunc) {
            compileEC();
        }
        
        ecFunc(cameraInfo.width * cameraInfo.height / 16, (*cameraImage)[0], ecImage.grayscaled[0], ecImage.saturated[0], ecImage.hued[0]);

        if(extractChroma) {
            extractChromaticity(cameraImage, cameraInfo, ecImage);
        }
        
        ecImage.timestamp = cameraImage->timestamp;
    }
}

void ECImageMiddleInfo::extractChromaticity(const CameraImage* cameraImage, const CameraInfo& cameraInfo, ECImage& eCImage)
{
    ASSERT(cameraImage->width == static_cast<unsigned int>(cameraInfo.width / 2));

    PixelTypes::GrayscaledPixel* uPos = eCImage.blueChromaticity[0];
    PixelTypes::GrayscaledPixel* vPos = eCImage.redChromaticity[0];
    const PixelTypes::YUYVPixel* yuyvUpperRowPos = (*cameraImage)[0];
    const PixelTypes::YUYVPixel* yuyvLowerRowPos = (*cameraImage)[0] + cameraImage->width;
    // += cameraImage->width to alternate iterating and skipping rows
    for(unsigned int yPos = 0; yPos < cameraImage->height; yPos += 2, yuyvUpperRowPos += cameraImage->width, yuyvLowerRowPos += cameraImage->width) {
        for(unsigned int xPos = 0; xPos < cameraImage->width; ++xPos, ++yuyvUpperRowPos, ++yuyvLowerRowPos, ++uPos, ++vPos) {
            *uPos = (yuyvUpperRowPos->u + yuyvLowerRowPos->u) >> 1;
            *vPos = (yuyvUpperRowPos->v + yuyvLowerRowPos->v) >> 1;
        }
    }
}

void ECImageMiddleInfo::compileE() {
    ASSERT(!eFunc);

    // Initialize assembler
    asmjit::CodeHolder code;
    code.init(this->jitRuntime->environment());
    asmjit::x86::Assembler a(&code);

    // Emit prolog
    a.enter(asmjit::imm(0u), asmjit::imm(0u));
#ifdef WINDOWS
    // Windows64
    asmjit::x86::Gp src = a.zdx();
    asmjit::x86::Gp dest = asmjit::x86::r8;
#else
    // System V x64
    a.mov(a.zcx(), a.zdi());
    asmjit::x86::Gp src = a.zsi();
    asmjit::x86::Gp dest = a.zdx();
#endif

    asmjit::Label loMask16 = a.newLabel();
    a.movdqa(asmjit::x86::xmm2, asmjit::x86::ptr(loMask16));

    asmjit::Label loop = a.newLabel();
    a.bind(loop);

    a.movdqu(asmjit::x86::xmm0, asmjit::x86::ptr(src, 0));
    a.movdqu(asmjit::x86::xmm1, asmjit::x86::ptr(src, 16));

    a.pand(asmjit::x86::xmm0, asmjit::x86::xmm2);
    a.pand(asmjit::x86::xmm1, asmjit::x86::xmm2);
    a.packuswb(asmjit::x86::xmm0, asmjit::x86::xmm1);

    a.add(src, asmjit::imm(16u * 2u));

    a.movdqa(asmjit::x86::ptr(dest), asmjit::x86::xmm0);
    a.add(dest, asmjit::imm(16u));

    a.dec(a.zcx());
    a.jnz(loop);

    // Emit epilog
    a.leave();
    a.ret();

    // Store constant
    a.align(asmjit::AlignMode::kZero, 16);
    a.bind(loMask16);
    a.embedUInt16(0x00FF, 8);

    // Bind function
    const asmjit::Error err = this->jitRuntime->add<EFunc>(&eFunc, &code);
    if(err) {
        // llog(INFO) << NDEBUG_LOGSYMB << err << std::endl;
        eFunc = nullptr;
    }
}

void ECImageMiddleInfo::compileEC() {
    ASSERT(!ecFunc);

    // Initialize assembler
    asmjit::CodeHolder code;
    code.init(this->jitRuntime->environment());
    asmjit::x86::Assembler a(&code);

    // Define argument registers
    asmjit::x86::Gp remainingSteps = asmjit::x86::edi;
    asmjit::x86::Gp src = a.zsi();
    asmjit::x86::Gp grayscaled = a.zdx();
    asmjit::x86::Gp saturated = a.zcx();
    asmjit::x86::Gp hued = a.zax();

    // Emit Prolog
    a.push(a.zbp());
    a.mov(a.zbp(), a.zsp());
#ifdef WINDOWS
    // Windows64
    a.push(a.zdi());
    a.push(a.zsi());
    a.mov(remainingSteps, asmjit::x86::ecx);
    a.mov(src, a.zdx());
    a.mov(grayscaled, asmjit::x86::r8);
    a.mov(saturated, asmjit::x86::r9);
    a.mov(hued, asmjit::x86::Mem(a.zbp(), 16 + 32));
#else
    // System V x64
    a.mov(hued, asmjit::x86::r8);
#endif

    // Define constants
    asmjit::Label constants = a.newLabel();
    asmjit::x86::Mem loMask16(constants, 0);
    asmjit::x86::Mem c8_128(constants, 16);
    asmjit::x86::Mem loMask32(constants, 16 * 2);
    asmjit::x86::Mem tallyInit(constants, 16 * 3);
    asmjit::x86::Mem c16_64(constants, 16 * 4);
    asmjit::x86::Mem c16_128(constants, 16 * 5);
    asmjit::x86::Mem c16_x8001(constants, 16 * 6);
    asmjit::x86::Mem c16_5695(constants, 16 * 7);
    asmjit::x86::Mem c16_11039(constants, 16 * 8);

    // Start of loop
    asmjit::Label loop = a.newLabel();
    a.bind(loop);
    // XMM0-XMM1: Source
    a.movdqu(asmjit::x86::xmm0, asmjit::x86::Mem(src, 0));
    a.movdqu(asmjit::x86::xmm1, asmjit::x86::Mem(src, 16));
    a.add(src, 32);

    // Compute luminance
    a.movdqa(asmjit::x86::xmm4, loMask16); // XMM4 is now loMask16
    a.movdqa(asmjit::x86::xmm2, asmjit::x86::xmm0);
    a.movdqa(asmjit::x86::xmm3, asmjit::x86::xmm1);
    a.pand(asmjit::x86::xmm2, asmjit::x86::xmm4); // XMM2 is now 16-bit luminance0
    a.pand(asmjit::x86::xmm3, asmjit::x86::xmm4); // XMM3 is now 16-bit luminance1
    a.movdqa(asmjit::x86::xmm5, asmjit::x86::xmm2);
    a.packuswb(asmjit::x86::xmm5, asmjit::x86::xmm3);
    // store grayscaled
    a.movdqa(asmjit::x86::ptr(grayscaled), asmjit::x86::xmm5);
    a.add(grayscaled, 16);

    // Convert image data to 8-bit UV in XMM0
    a.psrldq(asmjit::x86::xmm0, 1);
    a.psrldq(asmjit::x86::xmm1, 1);
    a.pand(asmjit::x86::xmm0, asmjit::x86::xmm4);
    a.pand(asmjit::x86::xmm1, asmjit::x86::xmm4);
    a.packuswb(asmjit::x86::xmm0, asmjit::x86::xmm1);
    a.psubb(asmjit::x86::xmm0, c8_128);

    // Compute saturation
    a.pabsb(asmjit::x86::xmm1, asmjit::x86::xmm0);
    a.pmaddubsw(asmjit::x86::xmm1, asmjit::x86::xmm1);
    a.pxor(asmjit::x86::xmm4, asmjit::x86::xmm4);
    a.punpcklwd(asmjit::x86::xmm4, asmjit::x86::xmm1);
    a.pslld(asmjit::x86::xmm4, 1);
    a.cvtdq2ps(asmjit::x86::xmm4, asmjit::x86::xmm4);
    a.rsqrtps(asmjit::x86::xmm4, asmjit::x86::xmm4); // XMM4 is now rnormUV0
    a.movdqa(asmjit::x86::xmm5, asmjit::x86::xmm2);
    a.movdqa(asmjit::x86::xmm6, asmjit::x86::xmm3);
    a.psrld(asmjit::x86::xmm5, 16); // XMM5 is now y1
    a.psrld(asmjit::x86::xmm6, 16); // XMM6 is now y3
    a.movdqa(asmjit::x86::xmm7, loMask32); // XMM7 is now loMask32
    a.pand(asmjit::x86::xmm2, asmjit::x86::xmm7); // XMM2 is now y0
    a.pand(asmjit::x86::xmm3, asmjit::x86::xmm7); // XMM3 is now y2
    a.cvtdq2ps(asmjit::x86::xmm2, asmjit::x86::xmm2);
    a.cvtdq2ps(asmjit::x86::xmm5, asmjit::x86::xmm5);
    a.cvtdq2ps(asmjit::x86::xmm3, asmjit::x86::xmm3);
    a.cvtdq2ps(asmjit::x86::xmm6, asmjit::x86::xmm6);
    a.mulps(asmjit::x86::xmm2, asmjit::x86::xmm4);
    a.mulps(asmjit::x86::xmm5, asmjit::x86::xmm4);
    a.rcpps(asmjit::x86::xmm2, asmjit::x86::xmm2);
    a.rcpps(asmjit::x86::xmm5, asmjit::x86::xmm5);
    a.cvtps2dq(asmjit::x86::xmm2, asmjit::x86::xmm2);
    a.cvtps2dq(asmjit::x86::xmm5, asmjit::x86::xmm5);
    a.pslld(asmjit::x86::xmm5, 16);
    a.por(asmjit::x86::xmm2, asmjit::x86::xmm5); // XMM2 is now 16-bit sat0
    a.pxor(asmjit::x86::xmm4, asmjit::x86::xmm4);
    a.punpckhwd(asmjit::x86::xmm4, asmjit::x86::xmm1);
    a.pslld(asmjit::x86::xmm4, 1);
    a.cvtdq2ps(asmjit::x86::xmm4, asmjit::x86::xmm4);
    a.rsqrtps(asmjit::x86::xmm4, asmjit::x86::xmm4); // XMM4 is now rnormUV1
    a.mulps(asmjit::x86::xmm3, asmjit::x86::xmm4);
    a.mulps(asmjit::x86::xmm6, asmjit::x86::xmm4);
    a.rcpps(asmjit::x86::xmm3, asmjit::x86::xmm3);
    a.rcpps(asmjit::x86::xmm6, asmjit::x86::xmm6);
    a.cvtps2dq(asmjit::x86::xmm3, asmjit::x86::xmm3);
    a.cvtps2dq(asmjit::x86::xmm6, asmjit::x86::xmm6);
    a.pslld(asmjit::x86::xmm6, 16);
    a.por(asmjit::x86::xmm3, asmjit::x86::xmm6); // XMM3 is now 16-bit sat1
    a.packuswb(asmjit::x86::xmm2, asmjit::x86::xmm3); // XMM2 is now 8-bit saturation
    // store saturated
    a.movntdq(asmjit::x86::ptr(saturated), asmjit::x86::xmm2);
    a.add(saturated, 16);

    // Compute hue
    a.movdqa(asmjit::x86::xmm1, asmjit::x86::xmm0);
    a.psraw(asmjit::x86::xmm1, 8); // XMM1 is now 16-bit V
    a.psllw(asmjit::x86::xmm0, 8);
    a.psraw(asmjit::x86::xmm0, 8); // XMM0 is now 16-bit U
    a.pabsw(asmjit::x86::xmm3, asmjit::x86::xmm0); // XMM3 is now 16-bit abs(U)
    a.pabsw(asmjit::x86::xmm4, asmjit::x86::xmm1); // XMM4 is now 16-bit abs(V)
    a.movdqa(asmjit::x86::xmm5, asmjit::x86::xmm3);
    a.pminsw(asmjit::x86::xmm5, asmjit::x86::xmm4); // XMM5 is now 16-bit min(abs(U),abs(V))
    a.pmaxsw(asmjit::x86::xmm3, asmjit::x86::xmm4); // XMM3 is now 16-bit max(abs(U),abs(V))
    a.pcmpeqw(asmjit::x86::xmm4, asmjit::x86::xmm5); // XMM4 is now (U > V)
    a.movdqa(asmjit::x86::xmm6, asmjit::x86::xmm0);
    a.psignw(asmjit::x86::xmm6, asmjit::x86::xmm1); // XMM6 is now sign(U,V)
    a.movdqa(asmjit::x86::xmm7, c16_128); // XMM7 is now c16_128
    a.pand(asmjit::x86::xmm0, asmjit::x86::xmm7);
    a.pand(asmjit::x86::xmm1, asmjit::x86::xmm7);
    a.pand(asmjit::x86::xmm0, asmjit::x86::xmm4);
    a.por(asmjit::x86::xmm1, c16_64);
    a.movdqa(asmjit::x86::xmm7, asmjit::x86::xmm4);
    a.pandn(asmjit::x86::xmm7, asmjit::x86::xmm1);
    a.por(asmjit::x86::xmm0, asmjit::x86::xmm7); // XMM0 is now the 16-bit atan2-offset
    a.pxor(asmjit::x86::xmm4, c16_x8001);
    a.psignw(asmjit::x86::xmm4, asmjit::x86::xmm6); // XMM4 is now the 16-bit atan2-sign
    // Scale and divide min by max
    a.movdqa(asmjit::x86::xmm6, tallyInit); // XMM6 is tally
    a.pxor(asmjit::x86::xmm1, asmjit::x86::xmm1); // XMM1 is quotient
    a.psllw(asmjit::x86::xmm3, 5);
    a.psllw(asmjit::x86::xmm5, 6);
    for(size_t i = 0; i < 5; i++) {
        a.movdqa(asmjit::x86::xmm7, asmjit::x86::xmm5);
        a.pcmpgtw(asmjit::x86::xmm7, asmjit::x86::xmm3); // XMM7 is now (min > max)
        a.pand(asmjit::x86::xmm7, asmjit::x86::xmm6);
        a.paddsw(asmjit::x86::xmm1, asmjit::x86::xmm7);
        a.movdqa(asmjit::x86::xmm7, asmjit::x86::xmm5);
        a.pcmpgtw(asmjit::x86::xmm7, asmjit::x86::xmm3); // XMM7 is now (min > max)
        a.pand(asmjit::x86::xmm7, asmjit::x86::xmm3);
        a.psubw(asmjit::x86::xmm5, asmjit::x86::xmm7);
        a.psrlw(asmjit::x86::xmm6, 1);
        a.psrlw(asmjit::x86::xmm3, 1);
    }
    // XMM1 is now (min << 15) / max
    a.movdqa(asmjit::x86::xmm3, asmjit::x86::xmm1);
    a.pmulhrsw(asmjit::x86::xmm3, c16_5695);
    a.movdqa(asmjit::x86::xmm5, c16_11039);
    a.psubw(asmjit::x86::xmm5, asmjit::x86::xmm3);
    a.pmulhrsw(asmjit::x86::xmm1, asmjit::x86::xmm5); // XMM1 is now the 16-bit absolute unrotated atan2
    a.psignw(asmjit::x86::xmm1, asmjit::x86::xmm4); // XMM1 is now the 16-bit unrotated atan2
    a.paddw(asmjit::x86::xmm0, asmjit::x86::xmm1); // XMM0 is now 16-bit hue
    a.psllw(asmjit::x86::xmm0, 8);
    a.movdqa(asmjit::x86::xmm1, asmjit::x86::xmm0);
    a.psrlw(asmjit::x86::xmm1, 8);
    a.por(asmjit::x86::xmm0, asmjit::x86::xmm1); // XMM0 is now 8-bit hue
    // store hued
    a.movntdq(asmjit::x86::ptr(hued), asmjit::x86::xmm0);
    a.add(hued, 16);

    // End of loop
    a.dec(remainingSteps);
    a.jnz(loop);

    // Return
#ifdef WINDOWS
    a.pop(a.zsi());
    a.pop(a.zdi());
#endif
    a.mov(a.zsp(), a.zbp());
    a.pop(a.zbp());
    a.ret();

    // Constants
    a.align(asmjit::AlignMode::kZero, 16);
    a.bind(constants);
    a.embedUInt16(0x00FF, 8);     // 0: loMask16
    a.embedUInt8(128, 16);        // 1: c8_128
    a.embedUInt32(0x0000FFFF, 4); // 2: loMask32
    a.embedUInt16(1 << 5, 8);     // 3: init for tally
    a.embedUInt16(64, 8);         // 4: c16_64
    a.embedUInt16(128, 8);        // 5: c16_128
    a.embedUInt16(0x8001, 8);     // 6: c16_x8001
    a.embedUInt16(5695, 8);       // 7: c16_5695
    a.embedUInt16(11039, 8);      // 8: c16_11039

    // Bind function
    const asmjit::Error err = this->jitRuntime->add<EcFunc>(&ecFunc, &code);
    if(err) {
        // llog(INFO) << NDEBUG_LOGSYMB << err << std::endl;
        ecFunc = nullptr;
        return;
    }
}

// #else

//#include "ImageProcessing/YHSColorConversion.h"

// template<bool aligned, bool avx>
// void updateSSE(const PixelTypes::YUYVPixel* const srcImage, const int srcWidth, const int srcHeight,
//                Image<PixelTypes::GrayscaledPixel>& grayscaled,
//                Image<PixelTypes::HuePixel>& hued, Image<PixelTypes::GrayscaledPixel>& saturated)
// {
//   ASSERT(srcWidth % 32 == 0);

//   __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(grayscaled[0]) - 1;
//   __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(saturated[0]) - 1;
//   __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(hued[0]) - 1;
//   const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(srcImage + srcWidth * srcHeight) - 1;

//   static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
//   static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

//   const char* prefetchSrc = reinterpret_cast<const char*>(srcImage) + (avx ? 128 : 64);
//   const char* prefetchGrayscaledDest = reinterpret_cast<const char*>(grayscaled[0]) + (avx ? 64 : 32);

//   const __m_auto_i* src = reinterpret_cast<__m_auto_i const*>(srcImage) - 1;
//   while(src < imageEnd)
//   {
//     const __m_auto_i p0 = _mmauto_loadt_si_all<aligned>(++src);
//     const __m_auto_i p1 = _mmauto_loadt_si_all<aligned>(++src);
//     const __m_auto_i p2 = _mmauto_loadt_si_all<aligned>(++src);
//     const __m_auto_i p3 = _mmauto_loadt_si_all<aligned>(++src);

//     // Compute luminance
//     const __m_auto_i y0 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p0, channelMask), _mmauto_and_si_all(p1, channelMask)));
//     const __m_auto_i y1 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p2, channelMask), _mmauto_and_si_all(p3, channelMask)));
//     _mmauto_storet_si_all<true>(++grayscaledDest, y0);
//     _mmauto_storet_si_all<true>(++grayscaledDest, y1);

//     _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
//     _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
//     if(avx) _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
//     if(avx) _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);

//     _mm_prefetch(prefetchGrayscaledDest += 32, _MM_HINT_T0);
//     if(avx) _mm_prefetch(prefetchGrayscaledDest += 32, _MM_HINT_T0);

//     // Compute saturation
//     const __m_auto_i uv0 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask))), c_128);
//     const __m_auto_i uv1 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p2, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p3, 1), channelMask))), c_128);

//     const __m_auto_i sat0 = YHSColorConversion::computeLightingIndependentSaturation<avx>(y0, uv0);
//     const __m_auto_i sat1 = YHSColorConversion::computeLightingIndependentSaturation<avx>(y1, uv1);
//     _mmauto_streamt_si_all<true>(++saturatedDest, sat0);
//     _mmauto_streamt_si_all<true>(++saturatedDest, sat1);

//     // Compute hue
//     const __m_auto_i hue = YHSColorConversion::computeHue<avx>(uv0, uv1);
//     __m_auto_i hue0 = hue;
//     __m_auto_i hue1 = hue;
//     _mmauto_unpacklohi_epi8(hue0, hue1);
//     _mmauto_streamt_si_all<true>(++huedDest, hue0);
//     _mmauto_streamt_si_all<true>(++huedDest, hue1);
//   }
// }

// TODO: Mark Putter - I don't think we need this 
// void ECImageMiddleInfo::detect_(CameraInfo::Camera whichCamera,
//   const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
//   const CameraInfo& upperInfo = info_in->cameraInfo[whichCamera];
  
//   ecImage.grayscaled.setResolution(CameraInfo.width, CameraInfo.height);
//   ecImage.saturated.setResolution(CameraInfo.width, CameraInfo.height);
//   ecImage.hued.setResolution(CameraInfo.width, CameraInfo.height);

//   if(theCameraImage.timestamp > 10 && static_cast<int>(theCameraImage.width) == CameraInfo.width / 2)
//   {
//     const PixelTypes::YUYVPixel* const src = reinterpret_cast<const PixelTypes::YUYVPixel* const>(theCameraImage[0]);
//     if(simdAligned<_supportsAVX2>(src))
//       updateSSE<true, _supportsAVX2>(src, theCameraImage.width, theCameraImage.height, ecImage.grayscaled, ecImage.hued, ecImage.saturated);
//     else
//       updateSSE<false, _supportsAVX2>(src, theCameraImage.width, theCameraImage.height, ecImage.grayscaled, ecImage.hued, ecImage.saturated);
//     ecImage.timestamp = theCameraImage.timestamp;
//   }
// }

#endif