{
  'includes': [
    '../../../../build/common.gypi',
  ],
  'targets': [
    {
      'target_name': 'webrtc_h261',
      'type': 'static_library',
      'dependencies': [
        '<(webrtc_root)/common_video/common_video.gyp:common_video',
        '<(webrtc_root)/modules/video_coding/utility/video_coding_utility.gyp:video_coding_utility',
        '<(webrtc_root)/system_wrappers/source/system_wrappers.gyp:system_wrappers',
      ],
      'include_dirs': [
        'include',
        '<(webrtc_root)/common_video/interface',
        '<(webrtc_root)/modules/video_coding/codecs/interface',
        '<(webrtc_root)/modules/interface',
      ],
      'direct_dependent_settings': {
        'include_dirs': [
          'include',
          '<(webrtc_root)/common_video/interface',
          '<(webrtc_root)/modules/video_coding/codecs/interface',
        ],
      },
      'sources': [
        'include/h261.h',
        'h261_impl.cc',
        'vic/bv.c',
        'vic/dct.cc',
        'vic/encoder-h261.cc',
        'vic/huffcode.cc',
        'vic/p64.cc',
        'vic/p64encoder.cc',
        'vic/transmitter.cc',
        'vic/vid_coder.cc',
      ],
      'cflags_mozilla': [
        '$(NSPR_CFLAGS)',
      ],
      'defines' : [
        'MOZILLA_INTERNAL_API',
      ],
    },
  ], # targets
}
