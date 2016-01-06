# Copyright 2016 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

{
  'targets': [
    {
      'target_name': 'openvr',
      'type': 'none',
      'include_dirs': [
        './openvr/headers',
      ],
      'conditions': [
        ['OS=="win"', {
          'msvs_settings': {
            'VCLinkerTool': {
              'AdditionalDependencies': [
                'openvr_api.lib',
              ],
              'AdditionalLibraryDirectories': [
                './openvr/lib/win32',
              ],
            },
          },
        }]
      ]
    },  # target openvr
  ]
}
