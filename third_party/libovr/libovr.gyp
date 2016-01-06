# Copyright 2016 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

{
  'targets': [
    {
      'target_name': 'libovr',
      'type': 'static_library',
      'include_dirs': [
        './LibOVR/Include',
      ],
      'sources': [
        'LibOVR/Src/OVR_CAPIShim.c',
      ],
    },  # target libovr
  ]
}
