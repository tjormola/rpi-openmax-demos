/*
 * Copyright © 2013 Tuomas Jormola <tj@solitudo.net> <http://solitudo.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * <http://www.apache.org/licenses/LICENSE-2.0>
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Short intro about this program:
 *
 * `rpi-encode-yuv` reads YUV planar 4:2:0 ([I420](http://www.fourcc.org/yuv.php#IYUV))
 * frame data from `stdin`, encodes the stream using the VideoCore hardware
 * encoder using H.264 codec and emits the H.264 stream to `stdout`.
 *
 *     $ ./rpi-encode-yuv <test.yuv >test.h264
 *
 * `rpi-encode-yuv` uses the `video_encode` component. Uncompressed YUV 4:2:0
 * ([I420](http://www.fourcc.org/yuv.php#IYUV)) frame data is read from `stdin`
 * and passed to the buffer of input port of `video_encode`. H.264 encoded video
 * is read from the buffer of `video_encode` output port and dumped to `stdout`.
 *
 * But similarly as described above in the [rpi-camera-dump-yuv] section, also
 * `video_encode` component requires its buffers to be formatted in
 * `OMX_COLOR_FormatYUV420PackedPlanar`. Thus we need to pack the I420 data to the
 * desired format while reading from input file and writing to `video_encode`
 * input buffer. Luckily no buffering is required here, you can just read the data
 * for each of the Y, U, and V planes directly to the `video_encode` input buffer
 * with proper alignment between the planes in the buffer.
 *
 * Please see README.mdwn for more detailed description of this
 * OpenMAX IL demos for Raspberry Pi bundle.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <bcm_host.h>

#include <interface/vcos/vcos_semaphore.h>
#include <interface/vmcs_host/vchost.h>

#include <IL/OMX_Core.h>
#include <IL/OMX_Component.h>
#include <IL/OMX_Video.h>
#include <IL/OMX_Broadcom.h>

// Hard coded parameters
#define VIDEO_WIDTH                     1920 / 4
#define VIDEO_HEIGHT                    1080 / 4
#define VIDEO_FRAMERATE                 25
#define VIDEO_BITRATE                   10000000

// Dunno where this is originally stolen from...
#define OMX_INIT_STRUCTURE(a) \
    memset(&(a), 0, sizeof(a)); \
    (a).nSize = sizeof(a); \
    (a).nVersion.nVersion = OMX_VERSION; \
    (a).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
    (a).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
    (a).nVersion.s.nRevision = OMX_VERSION_REVISION; \
    (a).nVersion.s.nStep = OMX_VERSION_STEP

// Global variable used by the signal handler and encoding loop
static int want_quit = 0;

// Our application context passed around
// the main routine and callback handlers
typedef struct {
    OMX_HANDLETYPE encoder;
    OMX_BUFFERHEADERTYPE *encoder_ppBuffer_in;
    OMX_BUFFERHEADERTYPE *encoder_ppBuffer_out;
    int encoder_input_buffer_needed;
    int encoder_output_buffer_available;
    int flushed;
    FILE *fd_in;
    FILE *fd_out;
    VCOS_SEMAPHORE_T handler_lock;
} appctx;

// I420 frame stuff
typedef struct {
    int width;
    int height;
    size_t size;
    int buf_stride;
    int buf_slice_height;
    int buf_extra_padding;
    int p_offset[3];
    int p_stride[3];
} i420_frame_info;

// Stolen from video-info.c of gstreamer-plugins-base
#define ROUND_UP_2(num) (((num)+1)&~1)
#define ROUND_UP_4(num) (((num)+3)&~3)
static void get_i420_frame_info(int width, int height, int buf_stride, int buf_slice_height, i420_frame_info *info) {
    info->p_stride[0] = ROUND_UP_4(width);
    info->p_stride[1] = ROUND_UP_4(ROUND_UP_2(width) / 2);
    info->p_stride[2] = info->p_stride[1];
    info->p_offset[0] = 0;
    info->p_offset[1] = info->p_stride[0] * ROUND_UP_2(height);
    info->p_offset[2] = info->p_offset[1] + info->p_stride[1] * (ROUND_UP_2(height) / 2);
    info->size = info->p_offset[2] + info->p_stride[2] * (ROUND_UP_2(height) / 2);
    info->width = width;
    info->height = height;
    info->buf_stride = buf_stride;
    info->buf_slice_height = buf_slice_height;
    info->buf_extra_padding =
        buf_slice_height >= 0
        ? ((buf_slice_height && (height % buf_slice_height))
             ? (buf_slice_height - (height % buf_slice_height))
             : 0)
        : -1;
}

// Ugly, stupid utility functions
static void say(const char* message, ...) {
    va_list args;
    char str[1024];
    memset(str, 0, sizeof(str));
    va_start(args, message);
    vsnprintf(str, sizeof(str) - 1, message, args);
    va_end(args);
    size_t str_len = strnlen(str, sizeof(str));
    if(str[str_len - 1] != '\n') {
        str[str_len] = '\n';
    }
    fprintf(stderr, str);
}

static void die(const char* message, ...) {
    va_list args;
    char str[1024];
    memset(str, 0, sizeof(str));
    va_start(args, message);
    vsnprintf(str, sizeof(str), message, args);
    va_end(args);
    say(str);
    exit(1);
}

static void omx_die(OMX_ERRORTYPE error, const char* message, ...) {
    va_list args;
    char str[1024];
    char *e;
    memset(str, 0, sizeof(str));
    va_start(args, message);
    vsnprintf(str, sizeof(str), message, args);
    va_end(args);
    switch(error) {
        case OMX_ErrorNone:                     e = "no error";                                      break;
        case OMX_ErrorBadParameter:             e = "bad parameter";                                 break;
        case OMX_ErrorIncorrectStateOperation:  e = "invalid state while trying to perform command"; break;
        case OMX_ErrorIncorrectStateTransition: e = "unallowed state transition";                    break;
        case OMX_ErrorInsufficientResources:    e = "insufficient resource";                         break;
        case OMX_ErrorBadPortIndex:             e = "bad port index, i.e. incorrect port";           break;
        case OMX_ErrorHardware:                 e = "hardware error";                                break;
        /* That's all I've encountered during hacking so let's not bother with the rest... */
        default:                                e = "(no description)";
    }
    die("OMX error: %s: 0x%08x %s", str, error, e);
}

static void dump_frame_info(const char *message, const i420_frame_info *info) {
    say("%s frame info:\n"
        "\tWidth:\t\t\t%d\n"
        "\tHeight:\t\t\t%d\n"
        "\tSize:\t\t\t%d\n"
        "\tBuffer stride:\t\t%d\n"
        "\tBuffer slice height:\t%d\n"
        "\tBuffer extra padding:\t%d\n"
        "\tPlane strides:\t\tY:%d U:%d V:%d\n"
        "\tPlane offsets:\t\tY:%d U:%d V:%d\n",
            message,
            info->width, info->height, info->size, info->buf_stride, info->buf_slice_height, info->buf_extra_padding,
            info->p_stride[0], info->p_stride[1], info->p_stride[2],
            info->p_offset[0], info->p_offset[1], info->p_offset[2]);
}

static void dump_event(OMX_HANDLETYPE hComponent, OMX_EVENTTYPE eEvent, OMX_U32 nData1, OMX_U32 nData2) {
    char *e;
    switch(eEvent) {
        case OMX_EventCmdComplete:          e = "command complete";                   break;
        case OMX_EventError:                e = "error";                              break;
        case OMX_EventParamOrConfigChanged: e = "parameter or configuration changed"; break;
        case OMX_EventPortSettingsChanged:  e = "port settings changed";              break;
        /* That's all I've encountered during hacking so let's not bother with the rest... */
        default:
            e = "(no description)";
    }
    say("Received event 0x%08x %s, hComponent:0x%08x, nData1:0x%08x, nData2:0x%08x",
            eEvent, e, hComponent, nData1, nData2);
}

static const char* dump_compression_format(OMX_VIDEO_CODINGTYPE c) {
    char *f;
    switch(c) {
        case OMX_VIDEO_CodingUnused:     return "not used";
        case OMX_VIDEO_CodingAutoDetect: return "autodetect";
        case OMX_VIDEO_CodingMPEG2:      return "MPEG2";
        case OMX_VIDEO_CodingH263:       return "H.263";
        case OMX_VIDEO_CodingMPEG4:      return "MPEG4";
        case OMX_VIDEO_CodingWMV:        return "Windows Media Video";
        case OMX_VIDEO_CodingRV:         return "RealVideo";
        case OMX_VIDEO_CodingAVC:        return "H.264/AVC";
        case OMX_VIDEO_CodingMJPEG:      return "Motion JPEG";
        case OMX_VIDEO_CodingVP6:        return "VP6";
        case OMX_VIDEO_CodingVP7:        return "VP7";
        case OMX_VIDEO_CodingVP8:        return "VP8";
        case OMX_VIDEO_CodingYUV:        return "Raw YUV video";
        case OMX_VIDEO_CodingSorenson:   return "Sorenson";
        case OMX_VIDEO_CodingTheora:     return "OGG Theora";
        case OMX_VIDEO_CodingMVC:        return "H.264/MVC";

        default:
            f = calloc(23, sizeof(char));
            if(f == NULL) {
                die("Failed to allocate memory");
            }
            snprintf(f, 23 * sizeof(char) - 1, "format type 0x%08x", c);
            return f;
    }
}
static const char* dump_color_format(OMX_COLOR_FORMATTYPE c) {
    char *f;
    switch(c) {
        case OMX_COLOR_FormatUnused:                 return "OMX_COLOR_FormatUnused: not used";
        case OMX_COLOR_FormatMonochrome:             return "OMX_COLOR_FormatMonochrome";
        case OMX_COLOR_Format8bitRGB332:             return "OMX_COLOR_Format8bitRGB332";
        case OMX_COLOR_Format12bitRGB444:            return "OMX_COLOR_Format12bitRGB444";
        case OMX_COLOR_Format16bitARGB4444:          return "OMX_COLOR_Format16bitARGB4444";
        case OMX_COLOR_Format16bitARGB1555:          return "OMX_COLOR_Format16bitARGB1555";
        case OMX_COLOR_Format16bitRGB565:            return "OMX_COLOR_Format16bitRGB565";
        case OMX_COLOR_Format16bitBGR565:            return "OMX_COLOR_Format16bitBGR565";
        case OMX_COLOR_Format18bitRGB666:            return "OMX_COLOR_Format18bitRGB666";
        case OMX_COLOR_Format18bitARGB1665:          return "OMX_COLOR_Format18bitARGB1665";
        case OMX_COLOR_Format19bitARGB1666:          return "OMX_COLOR_Format19bitARGB1666";
        case OMX_COLOR_Format24bitRGB888:            return "OMX_COLOR_Format24bitRGB888";
        case OMX_COLOR_Format24bitBGR888:            return "OMX_COLOR_Format24bitBGR888";
        case OMX_COLOR_Format24bitARGB1887:          return "OMX_COLOR_Format24bitARGB1887";
        case OMX_COLOR_Format25bitARGB1888:          return "OMX_COLOR_Format25bitARGB1888";
        case OMX_COLOR_Format32bitBGRA8888:          return "OMX_COLOR_Format32bitBGRA8888";
        case OMX_COLOR_Format32bitARGB8888:          return "OMX_COLOR_Format32bitARGB8888";
        case OMX_COLOR_FormatYUV411Planar:           return "OMX_COLOR_FormatYUV411Planar";
        case OMX_COLOR_FormatYUV411PackedPlanar:     return "OMX_COLOR_FormatYUV411PackedPlanar: Planes fragmented when a frame is split in multiple buffers";
        case OMX_COLOR_FormatYUV420Planar:           return "OMX_COLOR_FormatYUV420Planar: Planar YUV, 4:2:0 (I420)";
        case OMX_COLOR_FormatYUV420PackedPlanar:     return "OMX_COLOR_FormatYUV420PackedPlanar: Planar YUV, 4:2:0 (I420), planes fragmented when a frame is split in multiple buffers";
        case OMX_COLOR_FormatYUV420SemiPlanar:       return "OMX_COLOR_FormatYUV420SemiPlanar, Planar YUV, 4:2:0 (NV12), U and V planes interleaved with first U value";
        case OMX_COLOR_FormatYUV422Planar:           return "OMX_COLOR_FormatYUV422Planar";
        case OMX_COLOR_FormatYUV422PackedPlanar:     return "OMX_COLOR_FormatYUV422PackedPlanar: Planes fragmented when a frame is split in multiple buffers";
        case OMX_COLOR_FormatYUV422SemiPlanar:       return "OMX_COLOR_FormatYUV422SemiPlanar";
        case OMX_COLOR_FormatYCbYCr:                 return "OMX_COLOR_FormatYCbYCr";
        case OMX_COLOR_FormatYCrYCb:                 return "OMX_COLOR_FormatYCrYCb";
        case OMX_COLOR_FormatCbYCrY:                 return "OMX_COLOR_FormatCbYCrY";
        case OMX_COLOR_FormatCrYCbY:                 return "OMX_COLOR_FormatCrYCbY";
        case OMX_COLOR_FormatYUV444Interleaved:      return "OMX_COLOR_FormatYUV444Interleaved";
        case OMX_COLOR_FormatRawBayer8bit:           return "OMX_COLOR_FormatRawBayer8bit";
        case OMX_COLOR_FormatRawBayer10bit:          return "OMX_COLOR_FormatRawBayer10bit";
        case OMX_COLOR_FormatRawBayer8bitcompressed: return "OMX_COLOR_FormatRawBayer8bitcompressed";
        case OMX_COLOR_FormatL2:                     return "OMX_COLOR_FormatL2";
        case OMX_COLOR_FormatL4:                     return "OMX_COLOR_FormatL4";
        case OMX_COLOR_FormatL8:                     return "OMX_COLOR_FormatL8";
        case OMX_COLOR_FormatL16:                    return "OMX_COLOR_FormatL16";
        case OMX_COLOR_FormatL24:                    return "OMX_COLOR_FormatL24";
        case OMX_COLOR_FormatL32:                    return "OMX_COLOR_FormatL32";
        case OMX_COLOR_FormatYUV420PackedSemiPlanar: return "OMX_COLOR_FormatYUV420PackedSemiPlanar: Planar YUV, 4:2:0 (NV12), planes fragmented when a frame is split in multiple buffers, U and V planes interleaved with first U value";
        case OMX_COLOR_FormatYUV422PackedSemiPlanar: return "OMX_COLOR_FormatYUV422PackedSemiPlanar: Planes fragmented when a frame is split in multiple buffers";
        case OMX_COLOR_Format18BitBGR666:            return "OMX_COLOR_Format18BitBGR666";
        case OMX_COLOR_Format24BitARGB6666:          return "OMX_COLOR_Format24BitARGB6666";
        case OMX_COLOR_Format24BitABGR6666:          return "OMX_COLOR_Format24BitABGR6666";
        case OMX_COLOR_Format32bitABGR8888:          return "OMX_COLOR_Format32bitABGR8888";
        case OMX_COLOR_Format8bitPalette:            return "OMX_COLOR_Format8bitPalette";
        case OMX_COLOR_FormatYUVUV128:               return "OMX_COLOR_FormatYUVUV128";
        case OMX_COLOR_FormatRawBayer12bit:          return "OMX_COLOR_FormatRawBayer12bit";
        case OMX_COLOR_FormatBRCMEGL:                return "OMX_COLOR_FormatBRCMEGL";
        case OMX_COLOR_FormatBRCMOpaque:             return "OMX_COLOR_FormatBRCMOpaque";
        case OMX_COLOR_FormatYVU420PackedPlanar:     return "OMX_COLOR_FormatYVU420PackedPlanar";
        case OMX_COLOR_FormatYVU420PackedSemiPlanar: return "OMX_COLOR_FormatYVU420PackedSemiPlanar";
        default:
            f = calloc(23, sizeof(char));
            if(f == NULL) {
                die("Failed to allocate memory");
            }
            snprintf(f, 23 * sizeof(char) - 1, "format type 0x%08x", c);
            return f;
    }
}

static void dump_portdef(OMX_PARAM_PORTDEFINITIONTYPE* portdef) {
    say("Port %d is %s, %s, buffers wants:%d needs:%d, size:%d, pop:%d, aligned:%d",
        portdef->nPortIndex,
        (portdef->eDir ==  OMX_DirInput ? "input" : "output"),
        (portdef->bEnabled == OMX_TRUE ? "enabled" : "disabled"),
        portdef->nBufferCountActual,
        portdef->nBufferCountMin,
        portdef->nBufferSize,
        portdef->bPopulated,
        portdef->nBufferAlignment);

    OMX_VIDEO_PORTDEFINITIONTYPE *viddef = &portdef->format.video;
    OMX_IMAGE_PORTDEFINITIONTYPE *imgdef = &portdef->format.image;
    switch(portdef->eDomain) {
        case OMX_PortDomainVideo:
            say("Video type:\n"
                "\tWidth:\t\t%d\n"
                "\tHeight:\t\t%d\n"
                "\tStride:\t\t%d\n"
                "\tSliceHeight:\t%d\n"
                "\tBitrate:\t%d\n"
                "\tFramerate:\t%.02f\n"
                "\tError hiding:\t%s\n"
                "\tCodec:\t\t%s\n"
                "\tColor:\t\t%s\n",
                viddef->nFrameWidth,
                viddef->nFrameHeight,
                viddef->nStride,
                viddef->nSliceHeight,
                viddef->nBitrate,
                ((float)viddef->xFramerate / (float)65536),
                (viddef->bFlagErrorConcealment == OMX_TRUE ? "yes" : "no"),
                dump_compression_format(viddef->eCompressionFormat),
                dump_color_format(viddef->eColorFormat));
            break;
        case OMX_PortDomainImage:
            say("Image type:\n"
                "\tWidth:\t\t%d\n"
                "\tHeight:\t\t%d\n"
                "\tStride:\t\t%d\n"
                "\tSliceHeight:\t%d\n"
                "\tError hiding:\t%s\n"
                "\tCodec:\t\t%s\n"
                "\tColor:\t\t%s\n",
                imgdef->nFrameWidth,
                imgdef->nFrameHeight,
                imgdef->nStride,
                imgdef->nSliceHeight,
                (imgdef->bFlagErrorConcealment == OMX_TRUE ? "yes" : "no"),
                dump_compression_format(imgdef->eCompressionFormat),
                dump_color_format(imgdef->eColorFormat));
            break;
        default:
            break;
    }
}

static void dump_port(OMX_HANDLETYPE hComponent, OMX_U32 nPortIndex, OMX_BOOL dumpformats) {
    OMX_ERRORTYPE r;
    OMX_PARAM_PORTDEFINITIONTYPE portdef;
    OMX_INIT_STRUCTURE(portdef);
    portdef.nPortIndex = nPortIndex;
    if((r = OMX_GetParameter(hComponent, OMX_IndexParamPortDefinition, &portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get port definition for port %d", nPortIndex);
    }
    dump_portdef(&portdef);
    if(dumpformats) {
        OMX_VIDEO_PARAM_PORTFORMATTYPE portformat;
        OMX_INIT_STRUCTURE(portformat);
        portformat.nPortIndex = nPortIndex;
        portformat.nIndex = 0;
        r = OMX_ErrorNone;
        say("Port %d supports these video formats:", nPortIndex);
        while(r == OMX_ErrorNone) {
        if((r = OMX_GetParameter(hComponent, OMX_IndexParamVideoPortFormat, &portformat)) == OMX_ErrorNone) {
                say("\t%s, compression: %s", dump_color_format(portformat.eColorFormat), dump_compression_format(portformat.eCompressionFormat));
                portformat.nIndex++;
            }
        }
    }
}

// Some busy loops to verify we're running in order
static void block_until_state_changed(OMX_HANDLETYPE hComponent, OMX_STATETYPE wanted_eState) {
    OMX_STATETYPE eState;
    int i = 0;
    while(i++ == 0 || eState != wanted_eState) {
        OMX_GetState(hComponent, &eState);
        if(eState != wanted_eState) {
            usleep(10000);
        }
    }
}

static void block_until_port_changed(OMX_HANDLETYPE hComponent, OMX_U32 nPortIndex, OMX_BOOL bEnabled) {
    OMX_ERRORTYPE r;
    OMX_PARAM_PORTDEFINITIONTYPE portdef;
    OMX_INIT_STRUCTURE(portdef);
    portdef.nPortIndex = nPortIndex;
    OMX_U32 i = 0;
    while(i++ == 0 || portdef.bEnabled != bEnabled) {
        if((r = OMX_GetParameter(hComponent, OMX_IndexParamPortDefinition, &portdef)) != OMX_ErrorNone) {
            omx_die(r, "Failed to get port definition");
        }
        if(portdef.bEnabled != bEnabled) {
            usleep(10000);
        }
    }
}

static void block_until_flushed(appctx *ctx) {
    int quit;
    while(!quit) {
        vcos_semaphore_wait(&ctx->handler_lock);
        if(ctx->flushed) {
            ctx->flushed = 0;
            quit = 1;
        }
        vcos_semaphore_post(&ctx->handler_lock);
        if(!quit) {
            usleep(10000);
        }
    }
}

static void init_component_handle(
        const char *name,
        OMX_HANDLETYPE* hComponent,
        OMX_PTR pAppData,
        OMX_CALLBACKTYPE* callbacks) {
    OMX_ERRORTYPE r;
    char fullname[32];

    // Get handle
    memset(fullname, 0, sizeof(fullname));
    strcat(fullname, "OMX.broadcom.");
    strncat(fullname, name, strlen(fullname) - 1);
    say("Initializing component %s", fullname);
    if((r = OMX_GetHandle(hComponent, fullname, pAppData, callbacks)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get handle for component %s", fullname);
    }

    // Disable ports
    OMX_INDEXTYPE types[] = {
        OMX_IndexParamAudioInit,
        OMX_IndexParamVideoInit,
        OMX_IndexParamImageInit,
        OMX_IndexParamOtherInit
    };
    OMX_PORT_PARAM_TYPE ports;
    OMX_INIT_STRUCTURE(ports);
    OMX_GetParameter(*hComponent, OMX_IndexParamVideoInit, &ports);

    int i;
    for(i = 0; i < 4; i++) {
        if(OMX_GetParameter(*hComponent, types[i], &ports) == OMX_ErrorNone) {
            OMX_U32 nPortIndex;
            for(nPortIndex = ports.nStartPortNumber; nPortIndex < ports.nStartPortNumber + ports.nPorts; nPortIndex++) {
                say("Disabling port %d of component %s", nPortIndex, fullname);
                if((r = OMX_SendCommand(*hComponent, OMX_CommandPortDisable, nPortIndex, NULL)) != OMX_ErrorNone) {
                    omx_die(r, "Failed to disable port %d of component %s", nPortIndex, fullname);
                }
                block_until_port_changed(*hComponent, nPortIndex, OMX_FALSE);
            }
        }
    }
}

// Global signal handler for trapping SIGINT, SIGTERM, and SIGQUIT
static void signal_handler(int signal) {
    want_quit = 1;
}

// OMX calls this handler for all the events it emits
static OMX_ERRORTYPE event_handler(
        OMX_HANDLETYPE hComponent,
        OMX_PTR pAppData,
        OMX_EVENTTYPE eEvent,
        OMX_U32 nData1,
        OMX_U32 nData2,
        OMX_PTR pEventData) {

    dump_event(hComponent, eEvent, nData1, nData2);

    appctx *ctx = (appctx *)pAppData;

    switch(eEvent) {
        case OMX_EventCmdComplete:
            vcos_semaphore_wait(&ctx->handler_lock);
            if(nData1 == OMX_CommandFlush) {
                ctx->flushed = 1;
            }
            vcos_semaphore_post(&ctx->handler_lock);
            break;
        case OMX_EventError:
            omx_die(nData1, "error event received");
            break;
        default:
            break;
    }

    return OMX_ErrorNone;
}

// Called by OMX when the encoder component requires
// the input buffer to be filled with YUV video data
static OMX_ERRORTYPE empty_input_buffer_done_handler(
        OMX_HANDLETYPE hComponent,
        OMX_PTR pAppData,
        OMX_BUFFERHEADERTYPE* pBuffer) {
    appctx *ctx = ((appctx*)pAppData);
    vcos_semaphore_wait(&ctx->handler_lock);
    // The main loop can now fill the buffer from input file
    ctx->encoder_input_buffer_needed = 1;
    vcos_semaphore_post(&ctx->handler_lock);
    return OMX_ErrorNone;
}

// Called by OMX when the encoder component has filled
// the output buffer with H.264 encoded video data
static OMX_ERRORTYPE fill_output_buffer_done_handler(
        OMX_HANDLETYPE hComponent,
        OMX_PTR pAppData,
        OMX_BUFFERHEADERTYPE* pBuffer) {
    appctx *ctx = ((appctx*)pAppData);
    vcos_semaphore_wait(&ctx->handler_lock);
    // The main loop can now flush the buffer to output file
    ctx->encoder_output_buffer_available = 1;
    vcos_semaphore_post(&ctx->handler_lock);
    return OMX_ErrorNone;
}

int main(int argc, char **argv) {
    bcm_host_init();

    OMX_ERRORTYPE r;

    if((r = OMX_Init()) != OMX_ErrorNone) {
        omx_die(r, "OMX initalization failed");
    }

    // Init context
    appctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    if(vcos_semaphore_create(&ctx.handler_lock, "handler_lock", 1) != VCOS_SUCCESS) {
        die("Failed to create handler lock semaphore");
    }

    // Init component handles
    OMX_CALLBACKTYPE callbacks;
    memset(&ctx, 0, sizeof(callbacks));
    callbacks.EventHandler    = event_handler;
    callbacks.EmptyBufferDone = empty_input_buffer_done_handler;
    callbacks.FillBufferDone  = fill_output_buffer_done_handler;

    init_component_handle("video_encode", &ctx.encoder, &ctx, &callbacks);

    say("Configuring encoder...");

    say("Default port definition for encoder input port 200");
    dump_port(ctx.encoder, 200, OMX_TRUE);
    say("Default port definition for encoder output port 201");
    dump_port(ctx.encoder, 201, OMX_TRUE);

    OMX_PARAM_PORTDEFINITIONTYPE encoder_portdef;
    OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex = 200;
    if((r = OMX_GetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get port definition for encoder input port 200");
    }
    encoder_portdef.format.video.nFrameWidth  = VIDEO_WIDTH;
    encoder_portdef.format.video.nFrameHeight = VIDEO_HEIGHT;
    encoder_portdef.format.video.xFramerate   = VIDEO_FRAMERATE << 16;
    // Stolen from gstomxvideodec.c of gst-omx
    encoder_portdef.format.video.nStride      = (encoder_portdef.format.video.nFrameWidth + encoder_portdef.nBufferAlignment - 1) & (~(encoder_portdef.nBufferAlignment - 1));
    encoder_portdef.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
    if((r = OMX_SetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to set port definition for encoder input port 200");
    }

    // Copy encoder input port definition as basis encoder output port definition
    OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex = 200;
    if((r = OMX_GetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get port definition for encoder input port 200");
    }
    encoder_portdef.nPortIndex = 201;
    encoder_portdef.format.video.eColorFormat = OMX_COLOR_FormatUnused;
    encoder_portdef.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
    // Which one is effective, this or the configuration just below?
    encoder_portdef.format.video.nBitrate     = VIDEO_BITRATE;
    if((r = OMX_SetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to set port definition for encoder output port 201");
    }
    // Configure bitrate
    OMX_VIDEO_PARAM_BITRATETYPE bitrate;
    OMX_INIT_STRUCTURE(bitrate);
    bitrate.eControlRate = OMX_Video_ControlRateVariable;
    bitrate.nTargetBitrate = encoder_portdef.format.video.nBitrate;
    bitrate.nPortIndex = 201;
    if((r = OMX_SetParameter(ctx.encoder, OMX_IndexParamVideoBitrate, &bitrate)) != OMX_ErrorNone) {
        omx_die(r, "Failed to set bitrate for encoder output port 201");
    }
    // Configure format
    OMX_VIDEO_PARAM_PORTFORMATTYPE format;
    OMX_INIT_STRUCTURE(format);
    format.nPortIndex = 201;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;
    if((r = OMX_SetParameter(ctx.encoder, OMX_IndexParamVideoPortFormat, &format)) != OMX_ErrorNone) {
        omx_die(r, "Failed to set video format for encoder output port 201");
    }

    // Switch components to idle state
    say("Switching state of the encoder component to idle...");
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandStateSet, OMX_StateIdle, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to switch state of the encoder component to idle");
    }
    block_until_state_changed(ctx.encoder, OMX_StateIdle);

    // Enable ports
    say("Enabling ports...");
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandPortEnable, 200, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to enable encoder input port 200");
    }
    block_until_port_changed(ctx.encoder, 200, OMX_TRUE);
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandPortEnable, 201, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to enable encoder output port 201");
    }
    block_until_port_changed(ctx.encoder, 201, OMX_TRUE);

    // Allocate encoder input and output buffers
    say("Allocating buffers...");
    OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex = 200;
    if((r = OMX_GetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get port definition for encoder input port 200");
    }
    if((r = OMX_AllocateBuffer(ctx.encoder, &ctx.encoder_ppBuffer_in, 200, NULL, encoder_portdef.nBufferSize)) != OMX_ErrorNone) {
        omx_die(r, "Failed to allocate buffer for encoder input port 200");
    }
    OMX_INIT_STRUCTURE(encoder_portdef);
    encoder_portdef.nPortIndex = 201;
    if((r = OMX_GetParameter(ctx.encoder, OMX_IndexParamPortDefinition, &encoder_portdef)) != OMX_ErrorNone) {
        omx_die(r, "Failed to get port definition for encoder output port 201");
    }
    if((r = OMX_AllocateBuffer(ctx.encoder, &ctx.encoder_ppBuffer_out, 201, NULL, encoder_portdef.nBufferSize)) != OMX_ErrorNone) {
        omx_die(r, "Failed to allocate buffer for encoder output port 201");
    }

    // Just use stdin for input and stdout for output
    say("Opening input and output files...");
    ctx.fd_in = stdin;
    ctx.fd_out = stdout;

    // Switch state of the components prior to starting
    // the video capture and encoding loop
    say("Switching state of the encoder component to executing...");
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandStateSet, OMX_StateExecuting, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to switch state of the encoder component to executing");
    }
    block_until_state_changed(ctx.encoder, OMX_StateExecuting);

    say("Configured port definition for encoder input port 200");
    dump_port(ctx.encoder, 200, OMX_FALSE);
    say("Configured port definition for encoder output port 201");
    dump_port(ctx.encoder, 201, OMX_FALSE);

    i420_frame_info frame_info, buf_info;
    get_i420_frame_info(encoder_portdef.format.image.nFrameWidth, encoder_portdef.format.image.nFrameHeight, encoder_portdef.format.image.nStride, encoder_portdef.format.video.nSliceHeight, &frame_info);
    get_i420_frame_info(frame_info.buf_stride, frame_info.buf_slice_height, -1, -1, &buf_info);

    dump_frame_info("Destination frame", &frame_info);
    dump_frame_info("Source buffer", &buf_info);

    if(ctx.encoder_ppBuffer_in->nAllocLen != buf_info.size) {
        die("Allocated encoder input port 200 buffer size %d doesn't equal to the expected buffer size %d", ctx.encoder_ppBuffer_in->nAllocLen, buf_info.size);
    }

    say("Enter encode loop, press Ctrl-C to quit...");

    int input_available = 1, frame_in = 0, frame_out = 0, i;
    size_t input_total_read, want_read, input_read, output_written;
    // I420 spec: U and V plane span size half of the size of the Y plane span size
    int plane_span_y = ROUND_UP_2(frame_info.height), plane_span_uv = plane_span_y / 2;

    ctx.encoder_input_buffer_needed = 1;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);

    while(1) {
        // empty_input_buffer_done_handler() has marked that there's
        // a need for a buffer to be filled by us
        if(ctx.encoder_input_buffer_needed && input_available) {
            input_total_read = 0;
            memset(ctx.encoder_ppBuffer_in->pBuffer, 0, ctx.encoder_ppBuffer_in->nAllocLen);
            // Pack Y, U, and V plane spans read from input file to the buffer
            for(i = 0; i < 3; i++) {
                want_read = frame_info.p_stride[i] * (i == 0 ? plane_span_y : plane_span_uv);
                input_read = fread(
                    ctx.encoder_ppBuffer_in->pBuffer + buf_info.p_offset[i],
                    1, want_read, ctx.fd_in);
                input_total_read += input_read;
                if(input_read != want_read) {
                    ctx.encoder_ppBuffer_in->nFlags = OMX_BUFFERFLAG_EOS;
                    want_quit = 1;
                    say("Input file EOF");
                    break;
                }
            }
            ctx.encoder_ppBuffer_in->nOffset = 0;
            ctx.encoder_ppBuffer_in->nFilledLen = (buf_info.size - frame_info.size) + input_total_read;
            frame_in++;
            say("Read from input file and wrote to input buffer %d/%d, frame %d", ctx.encoder_ppBuffer_in->nFilledLen, ctx.encoder_ppBuffer_in->nAllocLen, frame_in);
            // Mark input unavailable also if the signal handler was triggered
            if(want_quit) {
                input_available = 0;
            }
            if(input_total_read > 0) {
                ctx.encoder_input_buffer_needed = 0;
                if((r = OMX_EmptyThisBuffer(ctx.encoder, ctx.encoder_ppBuffer_in)) != OMX_ErrorNone) {
                    omx_die(r, "Failed to request emptying of the input buffer on encoder input port 200");
                }
            }
        }
        // fill_output_buffer_done_handler() has marked that there's
        // a buffer for us to flush
        if(ctx.encoder_output_buffer_available) {
            if(ctx.encoder_ppBuffer_out->nFlags & OMX_BUFFERFLAG_ENDOFFRAME) {
                frame_out++;
            }
            // Flush buffer to output file
            output_written = fwrite(ctx.encoder_ppBuffer_out->pBuffer + ctx.encoder_ppBuffer_out->nOffset, 1, ctx.encoder_ppBuffer_out->nFilledLen, ctx.fd_out);
            if(output_written != ctx.encoder_ppBuffer_out->nFilledLen) {
                die("Failed to write to output file: %s", strerror(errno));
            }
            say("Read from output buffer and wrote to output file %d/%d, frame %d", ctx.encoder_ppBuffer_out->nFilledLen, ctx.encoder_ppBuffer_out->nAllocLen, frame_out + 1);
        }
        if(ctx.encoder_output_buffer_available || !frame_out) {
            // Buffer flushed, request a new buffer to be filled by the encoder component
            ctx.encoder_output_buffer_available = 0;
            if((r = OMX_FillThisBuffer(ctx.encoder, ctx.encoder_ppBuffer_out)) != OMX_ErrorNone) {
                omx_die(r, "Failed to request filling of the output buffer on encoder output port 201");
            }
        }
        // Don't exit the loop until all the input frames have been encoded.
        // Out frame count is larger than in frame count because 2 header
        // frames are emitted in the beginning.
        if(want_quit && frame_out == frame_in) {
            break;
        }
        // Would be better to use signaling here but hey this works too
        usleep(10);
    }
    say("Cleaning up...");

    // Restore signal handlers
    signal(SIGINT,  SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    signal(SIGQUIT, SIG_DFL);

    // Flush the buffers on each component
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandFlush, 200, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to flush buffers of encoder input port 200");
    }
    block_until_flushed(&ctx);
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandFlush, 201, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to flush buffers of encoder output port 201");
    }
    block_until_flushed(&ctx);

    // Disable all the ports
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandPortDisable, 200, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to disable encoder input port 200");
    }
    block_until_port_changed(ctx.encoder, 200, OMX_FALSE);
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandPortDisable, 201, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to disable encoder output port 201");
    }
    block_until_port_changed(ctx.encoder, 201, OMX_FALSE);

    // Free all the buffers
    if((r = OMX_FreeBuffer(ctx.encoder, 200, ctx.encoder_ppBuffer_in)) != OMX_ErrorNone) {
        omx_die(r, "Failed to free buffer for encoder input port 200");
    }
    if((r = OMX_FreeBuffer(ctx.encoder, 201, ctx.encoder_ppBuffer_out)) != OMX_ErrorNone) {
        omx_die(r, "Failed to free buffer for encoder output port 201");
    }

    // Transition all the components to idle and then to loaded states
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandStateSet, OMX_StateIdle, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to switch state of the encoder component to idle");
    }
    block_until_state_changed(ctx.encoder, OMX_StateIdle);
    if((r = OMX_SendCommand(ctx.encoder, OMX_CommandStateSet, OMX_StateLoaded, NULL)) != OMX_ErrorNone) {
        omx_die(r, "Failed to switch state of the encoder component to loaded");
    }
    block_until_state_changed(ctx.encoder, OMX_StateLoaded);

    // Free the component handles
    if((r = OMX_FreeHandle(ctx.encoder)) != OMX_ErrorNone) {
        omx_die(r, "Failed to free encoder component handle");
    }

    // Exit
    fclose(ctx.fd_in);
    fclose(ctx.fd_out);

    vcos_semaphore_delete(&ctx.handler_lock);
    if((r = OMX_Deinit()) != OMX_ErrorNone) {
        omx_die(r, "OMX de-initalization failed");
    }

    say("Exit!");

    return 0;
}
