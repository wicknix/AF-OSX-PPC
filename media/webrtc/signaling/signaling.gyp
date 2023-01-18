# Copyright (c) 2011 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# Indent 2 spaces, no tabs.
#
#
# sip.gyp - a library for SIP
#

{
  'variables': {
    'chromium_code': 1,
  },

  'target_defaults': {
    'conditions': [
      ['moz_widget_toolkit_gonk==1', {
        'defines' : [
          'WEBRTC_GONK',
       ],
      }],
    ],
  },

  'targets': [

    #
    # ECC
    #
    {
      'target_name': 'ecc',
      'type': 'static_library',

      #
      # INCLUDES
      #
      'include_dirs': [
        '..',
        './src',
        './src/common',
        './src/common/browser_logging',
        './src/common/time_profiling',
        './src/media',
        './src/media-conduit',
        './src/mediapipeline',
        './src/peerconnection',
        './src/sdp/sipcc',
        '../../../xpcom/base',
        '../../../dom/base',
        '../../../dom/media',
        '../../../media/mtransport',
        '../trunk',
        '../../libyuv/include',
        '../../mtransport/third_party/nrappkit/src/util/libekr',
      ],

      #
      # DEPENDENCIES
      #
      'dependencies': [
      ],

      'export_dependent_settings': [
      ],


      #
      # SOURCES
      #
      'sources': [
        # Media Conduit
        './src/media-conduit/AudioConduit.h',
        './src/media-conduit/AudioConduit.cpp',
        './src/media-conduit/VideoConduit.h',
        './src/media-conduit/VideoConduit.cpp',
        './src/media-conduit/CodecStatistics.h',
        './src/media-conduit/CodecStatistics.cpp',
        './src/media-conduit/RunningStat.h',
        './src/media-conduit/GmpVideoCodec.cpp',
        './src/media-conduit/WebrtcGmpVideoCodec.cpp',
        # Common
        './src/common/CommonTypes.h',
        './src/common/csf_common.h',
        './src/common/NullDeleter.h',
        './src/common/Wrapper.h',
        './src/common/NullTransport.h',
        './src/common/YuvStamper.cpp',
        # Browser Logging
        './src/common/browser_logging/CSFLog.cpp',
        './src/common/browser_logging/CSFLog.h',
        './src/common/browser_logging/WebRtcLog.cpp',
        './src/common/browser_logging/WebRtcLog.h',
        # Browser Logging
        './src/common/time_profiling/timecard.c',
        './src/common/time_profiling/timecard.h',
        # PeerConnection
        './src/peerconnection/MediaPipelineFactory.cpp',
        './src/peerconnection/MediaPipelineFactory.h',
        './src/peerconnection/MediaStreamList.cpp',
        './src/peerconnection/MediaStreamList.h',
        './src/peerconnection/PeerConnectionCtx.cpp',
        './src/peerconnection/PeerConnectionCtx.h',
        './src/peerconnection/PeerConnectionImpl.cpp',
        './src/peerconnection/PeerConnectionImpl.h',
        './src/peerconnection/PeerConnectionMedia.cpp',
        './src/peerconnection/PeerConnectionMedia.h',
        # Media pipeline
        './src/mediapipeline/MediaPipeline.h',
        './src/mediapipeline/MediaPipeline.cpp',
        './src/mediapipeline/MediaPipelineFilter.h',
        './src/mediapipeline/MediaPipelineFilter.cpp',
         # SDP
         './src/sdp/sipcc/ccsdp.h',
         './src/sdp/sipcc/cpr_string.c',
         './src/sdp/sipcc/sdp_access.c',
         './src/sdp/sipcc/sdp_attr.c',
         './src/sdp/sipcc/sdp_attr_access.c',
         './src/sdp/sipcc/sdp_base64.c',
         './src/sdp/sipcc/sdp_config.c',
         './src/sdp/sipcc/sdp_main.c',
         './src/sdp/sipcc/sdp_token.c',
         './src/sdp/sipcc/sdp.h',
         './src/sdp/sipcc/sdp_base64.h',
         './src/sdp/sipcc/sdp_os_defs.h',
         './src/sdp/sipcc/sdp_private.h',
         './src/sdp/sipcc/sdp_utils.c',
         './src/sdp/sipcc/sdp_services_unix.c',

         # SDP Wrapper
         './src/sdp/Sdp.h',
         './src/sdp/SdpAttribute.h',
         './src/sdp/SdpAttribute.cpp',
         './src/sdp/SdpAttributeList.h',
         './src/sdp/SdpErrorHolder.h',
         './src/sdp/SdpMediaSection.h',
         './src/sdp/SipccSdp.h',
         './src/sdp/SipccSdpAttributeList.h',
         './src/sdp/SipccSdpAttributeList.cpp',
         './src/sdp/SipccSdpMediaSection.h',
         './src/sdp/SipccSdpParser.h',
         './src/sdp/SipccSdp.cpp',
         './src/sdp/SipccSdpMediaSection.cpp',
         './src/sdp/SipccSdpParser.cpp',

         # JSEP
         './src/jsep/JsepCodecDescription.h',
         './src/jsep/JsepSession.h',
         './src/jsep/JsepSessionImpl.cpp',
         './src/jsep/JsepSessionImpl.h',
         './src/jsep/JsepTrack.h',
         './src/jsep/JsepTransport.h'
      ],

      #
      # DEFINES
      #

      'defines' : [
        'LOG4CXX_STATIC',
        '_NO_LOG4CXX',
        'USE_SSLEAY',
        '_CPR_USE_EXTERNAL_LOGGER',
        'WEBRTC_RELATIVE_PATH',
        'HAVE_WEBRTC_VIDEO',
        'HAVE_WEBRTC_VOICE',
        'HAVE_STDINT_H=1',
        'HAVE_STDLIB_H=1',
        'HAVE_UINT8_T=1',
        'HAVE_UINT16_T=1',
        'HAVE_UINT32_T=1',
        'HAVE_UINT64_T=1',
      ],

      'cflags_mozilla': [
        '$(NSPR_CFLAGS)',
        '$(NSS_CFLAGS)',
        '$(MOZ_PIXMAN_CFLAGS)',
        '$(WARNINGS_AS_ERRORS)',
      ],


      #
      # Conditionals
      #
      'conditions': [
        # hack so I can change the include flow for SrtpFlow
        ['build_with_mozilla==1', {
          'sources': [
            './src/mediapipeline/SrtpFlow.h',
            './src/mediapipeline/SrtpFlow.cpp',
          ],
          'include_dirs!': [
            '../trunk/webrtc',
          ],
          'include_dirs': [
            '../../../netwerk/srtp/src/include',
            '../../../netwerk/srtp/src/crypto/include',
          ],
        }],
        ['moz_webrtc_omx==1', {
          'sources': [
            './src/media-conduit/WebrtcOMXH264VideoCodec.cpp',
            './src/media-conduit/OMXVideoCodec.cpp',
          ],
          'include_dirs': [
            # hack on hack to re-add it after SrtpFlow removes it
            '../../../dom/media/omx',
            '../../../gfx/layers/client',
          ],
          'cflags_mozilla': [
            '-I$(ANDROID_SOURCE)/frameworks/av/include/media/stagefright',
            '-I$(ANDROID_SOURCE)/frameworks/av/include',
            '-I$(ANDROID_SOURCE)/frameworks/native/include/media/openmax',
            '-I$(ANDROID_SOURCE)/frameworks/native/include',
            '-I$(ANDROID_SOURCE)/frameworks/native/opengl/include',
          ],
          'defines' : [
            'MOZ_WEBRTC_OMX'
          ],
        }],
        ['build_for_test==0', {
          'defines' : [
            'MOZILLA_INTERNAL_API'
          ],
          'sources': [
            './src/peerconnection/WebrtcGlobalInformation.cpp',
            './src/peerconnection/WebrtcGlobalInformation.h',
          ],
        }],
        ['build_for_test!=0', {
          'include_dirs': [
            './test'
          ],
          'defines' : [
            'NO_CHROMIUM_LOGGING',
            'USE_FAKE_MEDIA_STREAMS',
            'USE_FAKE_PCOBSERVER'
          ],
        }],
        ['(OS=="linux") or (OS=="android")', {
          'include_dirs': [
          ],

          'defines': [
            'OS_LINUX',
            'SIP_OS_LINUX',
            'WEBRTC_POSIX',
            '_GNU_SOURCE',
            'LINUX',
            'GIPS_VER=3510',
            'SECLIB_OPENSSL',
          ],

          'cflags_mozilla': [
          ],
        }],
        ['OS=="android" or moz_widget_toolkit_gonk==1', {
          'cflags_mozilla': [
            # This warning complains about important MOZ_EXPORT attributes
            # on forward declarations for Android API types.
            '-Wno-error=attributes',
          ],
        }],
        ['OS=="win"', {
          'include_dirs': [
          ],
          'defines': [
            'OS_WIN',
            'SIP_OS_WINDOWS',
            'WIN32',
            'GIPS_VER=3480',
            'SIPCC_BUILD',
            'HAVE_WINSOCK2_H'
          ],

          'cflags_mozilla': [
          ],
        }],
        ['os_bsd==1', {
          'include_dirs': [
          ],
          'defines': [
            # avoiding pointless ifdef churn
            'WEBRTC_POSIX',
            'SIP_OS_OSX',
            'OSX',
            'SECLIB_OPENSSL',
          ],

          'cflags_mozilla': [
          ],
        }],
        ['OS=="mac"', {
          'include_dirs': [
          ],
          'defines': [
            'WEBRTC_POSIX',
            'OS_MACOSX',
            'SIP_OS_OSX',
            'OSX',
            '_FORTIFY_SOURCE=2',
          ],

          'cflags_mozilla': [
          ],
        }],
      ],
    },
  ],
}

# Local Variables:
# tab-width:2
# indent-tabs-mode:nil
# End:
# vim: set expandtab tabstop=2 shiftwidth=2:
